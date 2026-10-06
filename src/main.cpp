#include <Arduino.h>
#include <M5Unified.h>
#include <lvgl.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace {

// ---------------------------------------------------------------- wiring --
constexpr uint32_t CRSF_BAUD = 420000;
constexpr int CRSF_TX_PIN = 32;  // Core2 Port A SDA pin, wired to the CRSF RX pad.
constexpr int CRSF_RX_PIN = 33;  // Core2 Port A SCL pin, wired to the CRSF TX pad.
constexpr size_t CRSF_RX_BUFFER = 512;

// ------------------------------------------------------------------ CRSF --
constexpr uint8_t CRSF_SYNC_BROADCAST = 0x00;
constexpr uint8_t CRSF_SYNC_FLIGHT_CONTROLLER = 0xC8;
constexpr uint8_t CRSF_SYNC_RADIO = 0xEA;
constexpr uint8_t CRSF_SYNC_RECEIVER = 0xEC;
constexpr uint8_t CRSF_SYNC_TRANSMITTER = 0xEE;

constexpr uint8_t CRSF_TYPE_BATTERY_SENSOR = 0x08;
constexpr uint8_t CRSF_TYPE_RC_CHANNELS_PACKED = 0x16;

// The length byte counts type + payload + CRC, so a 62-byte maximum makes the
// longest frame on the wire 64 bytes including the sync and length bytes.
constexpr uint8_t CRSF_LEN_MIN = 2;
constexpr uint8_t CRSF_LEN_MAX = 62;
constexpr uint8_t CRSF_FRAME_MAX = CRSF_LEN_MAX + 2;

// RC_CHANNELS_PACKED is 16 channels of 11 bits each, and that is the only
// channel count the standard frame can carry.
constexpr uint8_t CHANNEL_COUNT = 16;
constexpr uint8_t RC_PAYLOAD_BYTES = 22;
constexpr uint8_t RC_FRAME_LENGTH = RC_PAYLOAD_BYTES + 2;  // type + payload + CRC
constexpr uint8_t RC_FRAME_BYTES = RC_PAYLOAD_BYTES + 4;   // sync + length + the above

constexpr uint16_t CHANNEL_MIN = 1000;
constexpr uint16_t CHANNEL_CENTER = 1500;
constexpr uint16_t CHANNEL_MAX = 2000;
constexpr int32_t CRSF_TICKS_MIN = 172;   // what 1000 us maps to on the wire
constexpr int32_t CRSF_TICKS_MAX = 1811;  // what 2000 us maps to on the wire

constexpr uint32_t FRAME_INTERVAL_MS = 20;  // 50 Hz, the usual CRSF RC rate
constexpr uint32_t POLL_INTERVAL_MS = 5;    // how often the link task drains the UART
constexpr uint8_t POLLS_PER_FRAME = FRAME_INTERVAL_MS / POLL_INTERVAL_MS;
constexpr uint32_t LINK_TIMEOUT_MS = 1000;
constexpr uint32_t RX_IDLE_RESYNC_MS = 10;  // a stalled partial frame is abandoned

// -------------------------------------------------------------- UI layout --
constexpr lv_coord_t SCREEN_W = 320;
constexpr lv_coord_t SCREEN_H = 240;
constexpr uint16_t DRAW_BUFFER_LINES = 40;

constexpr lv_coord_t HEADER_H = 26;
constexpr lv_coord_t FOOTER_H = 28;
// Not LIST_H: FreeRTOS list.h uses that name as its include guard.
constexpr lv_coord_t LIST_HEIGHT = SCREEN_H - HEADER_H - FOOTER_H;
// The name sits on its own line above the control. Four rows leave 44 px
// each, which is enough for a 14 px name and a control below it that is
// comfortable to hit, rather than the 8 px track the eight row layout forced.
constexpr lv_coord_t ROW_H = 44;
constexpr lv_coord_t ROW_GAP = 3;
// The digitizer is unreliable in the last few pixels at the bezel, so a slider
// running to the screen edge could not be driven to its end stop. The gutter
// keeps the track ends clear of the edge with room to overshoot them, which
// LVGL clamps to the end value.
constexpr lv_coord_t SCREEN_GUTTER = 10;
constexpr lv_coord_t ROW_W = SCREEN_W - 2 * SCREEN_GUTTER;
constexpr lv_coord_t NAME_X = 6;
constexpr lv_coord_t NAME_W = 200;
constexpr lv_coord_t NAME_Y = 4;
constexpr lv_coord_t VALUE_W = 60;
constexpr lv_coord_t SLIDER_X = 8;
constexpr lv_coord_t SLIDER_Y = 28;
constexpr lv_coord_t SLIDER_W = ROW_W - 2 * SLIDER_X;
constexpr lv_coord_t SLIDER_H = 10;
constexpr lv_coord_t SWITCH_X = 6;
constexpr lv_coord_t SWITCH_Y = 23;
constexpr lv_coord_t SWITCH_W = ROW_W - 2 * SWITCH_X;
constexpr lv_coord_t SWITCH_H = 18;

constexpr lv_coord_t BUTTON_W = 100;
// Centred over BtnB, so the bezel button and the on-screen one line up.
constexpr lv_coord_t RESET_BUTTON_X = (SCREEN_W - BUTTON_W) / 2;
constexpr lv_coord_t BUTTON_H = 26;

constexpr uint32_t COLOR_BG = 0x0E1621;
constexpr uint32_t COLOR_PANEL = 0x18222E;
constexpr uint32_t COLOR_ROW = 0x1B2633;
constexpr uint32_t COLOR_BUTTON = 0x2A3848;
constexpr uint32_t COLOR_TEXT = 0xE7EFF7;
constexpr uint32_t COLOR_TEXT_DIM = 0x8195A8;
constexpr uint32_t COLOR_TRACK = 0x3A4756;
constexpr uint32_t COLOR_INDICATOR = 0x27C2A2;
constexpr uint32_t COLOR_KNOB = 0xF4C95D;
constexpr uint32_t COLOR_CHANGED = 0xF4C95D;  // same amber as the slider knob
constexpr uint32_t COLOR_LINK_OK = 0x4ADE80;

enum ControlKind : uint8_t {
    CONTROL_SLIDER,  // anywhere from CHANNEL_MIN to CHANNEL_MAX
    CONTROL_SWITCH,  // only the evenly spaced positions listed below
};

// A switch channel sends nothing but its detent values: two positions give
// 1000/2000 and three give 1000/1500/2000, so a switch can never come to rest
// on an intermediate value the receiving end would have to interpret.
struct ChannelConfig {
    const char *name;        // empty falls back to "CH n"
    ControlKind control;
    uint8_t positions;       // switch channels only
    uint16_t defaultValue;   // what the channel holds at power on and on RESET
};

// Edit this table to re-map the panel. Only CH5 was specified as a two
// position switch; the other switch entries are inferred from their names,
// with SB/SC/SD read as the usual three position transmitter switches.
constexpr ChannelConfig CHANNELS[CHANNEL_COUNT] = {
    {"Aileron",     CONTROL_SLIDER, 0, CHANNEL_CENTER},
    {"Elevation",   CONTROL_SLIDER, 0, CHANNEL_CENTER},
    {"Thrust",      CONTROL_SLIDER, 0, CHANNEL_MIN},  // throttle rests at idle
    {"Rudder",      CONTROL_SLIDER, 0, CHANNEL_CENTER},
    {"Takeoff",     CONTROL_SWITCH, 2, CHANNEL_MIN},
    {"Air/Flight",  CONTROL_SWITCH, 2, CHANNEL_MIN},
    {"ARM",         CONTROL_SWITCH, 2, CHANNEL_MIN},  // must power on disarmed
    {"FIRE",        CONTROL_SWITCH, 2, CHANNEL_MIN},
    {"Cam Servo",   CONTROL_SWITCH, 3, CHANNEL_MAX},
    {"SD",          CONTROL_SWITCH, 3, CHANNEL_MIN},
    {"Contrast",    CONTROL_SLIDER, 0, CHANNEL_CENTER},
    {"Vid Channel", CONTROL_SWITCH, 6, CHANNEL_MIN},  // 1000-2000 in 200s
    {"SB",          CONTROL_SWITCH, 3, CHANNEL_MIN},
    {"",            CONTROL_SLIDER, 0, CHANNEL_CENTER},
    {"SC",          CONTROL_SWITCH, 3, CHANNEL_MIN},
    {"Tx Power",    CONTROL_SLIDER, 0, CHANNEL_CENTER},
};

// The only channels with a control on screen. Every other channel is still
// sent in every frame, pinned at its default, so the sticks sit centred with
// the throttle at idle.
constexpr uint8_t PANEL_CHANNELS[] = {6, 7, 8, 10};  // ARM, FIRE, Cam Servo, Contrast
constexpr uint8_t ROW_COUNT = sizeof(PANEL_CHANNELS);

// lv_btnmatrix keeps a pointer to the map it is given, so these must outlive
// the matrices built from them.
const char *SWITCH_MAP_2[] = {"LO", "HI", ""};
const char *SWITCH_MAP_3[] = {"LO", "MID", "HI", ""};
const char *SWITCH_MAP_6[] = {"1", "2", "3", "4", "5", "6", ""};

const char **switchMap(uint8_t positions) {
    switch (positions) {
        case 6:
            return SWITCH_MAP_6;
        case 3:
            return SWITCH_MAP_3;
        default:
            return SWITCH_MAP_2;
    }
}

// ------------------------------------------------------------------ state --
// UART2. Using the core's existing instance avoids a second HardwareSerial
// object bound to the same peripheral. The explicit pins passed to begin()
// keep it off the default 16/17, which the Core2 PSRAM uses.
HardwareSerial &crsfUart = Serial2;

// Written by the UI task on core 1 and read by the link task on core 0. Each
// entry is an aligned 16-bit word, so a reader always sees a whole value.
volatile uint16_t channelValues[CHANNEL_COUNT];

// Written by the link task and read by the UI task. Both are aligned 32-bit
// words, so neither can be seen half updated.
volatile uint32_t lastTelemetryMillis = 0;
volatile int32_t batteryDecivolts = -1;

lv_disp_draw_buf_t drawBuffer;
lv_color_t drawPixels[SCREEN_W * DRAW_BUFFER_LINES];
lv_disp_drv_t displayDriver;
lv_indev_drv_t touchDriver;

lv_obj_t *statusLabel;

struct RowWidgets {
    uint8_t channel;
    lv_obj_t *control;
    lv_obj_t *value;
};
RowWidgets rows[ROW_COUNT];

uint8_t channelForSlot(uint8_t slot) { return rows[slot].channel; }

uint16_t switchPositionValue(const ChannelConfig &config, uint8_t index) {
    if (config.positions < 2) {
        return config.defaultValue;
    }
    const uint32_t span = CHANNEL_MAX - CHANNEL_MIN;
    return static_cast<uint16_t>(CHANNEL_MIN + (span * index) / (config.positions - 1));
}

uint8_t switchIndexForValue(const ChannelConfig &config, uint16_t value) {
    for (uint8_t index = 0; index < config.positions; ++index) {
        if (switchPositionValue(config, index) == value) {
            return index;
        }
    }
    return 0;
}

// =========================================================== CRSF link ====

uint8_t crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0;
    while (length--) {
        crc ^= *data++;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xD5)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

void sendCrsfFrame() {
    uint8_t frame[RC_FRAME_BYTES] = {};
    frame[0] = CRSF_SYNC_FLIGHT_CONTROLLER;
    frame[1] = RC_FRAME_LENGTH;
    frame[2] = CRSF_TYPE_RC_CHANNELS_PACKED;

    // Channels are packed least significant bit first, so channel 1 occupies
    // bits 0-10 of the payload, channel 2 bits 11-21, and so on.
    uint32_t bits = 0;
    uint8_t bitCount = 0;
    uint8_t outputIndex = 3;
    for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
        const int32_t microseconds = static_cast<int32_t>(channelValues[channel]);
        // constrain() is a macro, so map() is evaluated into a local first.
        const int32_t scaled =
            map(microseconds, CHANNEL_MIN, CHANNEL_MAX, CRSF_TICKS_MIN, CRSF_TICKS_MAX);
        const int32_t ticks = constrain(scaled, CRSF_TICKS_MIN, CRSF_TICKS_MAX);
        bits |= static_cast<uint32_t>(ticks) << bitCount;
        bitCount += 11;
        while (bitCount >= 8) {
            frame[outputIndex++] = static_cast<uint8_t>(bits & 0xFF);
            bits >>= 8;
            bitCount -= 8;
        }
    }
    frame[outputIndex] = crc8(&frame[2], RC_FRAME_LENGTH - 1);
    crsfUart.write(frame, sizeof(frame));
}

bool isCrsfSyncByte(uint8_t byte) {
    return byte == CRSF_SYNC_FLIGHT_CONTROLLER || byte == CRSF_SYNC_RADIO ||
           byte == CRSF_SYNC_RECEIVER || byte == CRSF_SYNC_TRANSMITTER ||
           byte == CRSF_SYNC_BROADCAST;
}

void handleReceivedFrame(const uint8_t *frame) {
    const uint8_t length = frame[1];
    if (crc8(&frame[2], length - 1) != frame[length + 1]) {
        return;
    }

    lastTelemetryMillis = millis();
    // Battery sensor payload is voltage and current in tenths, then capacity
    // and remaining percent, so the length byte is type + 8 bytes + CRC.
    if (frame[2] == CRSF_TYPE_BATTERY_SENSOR && length >= 10) {
        batteryDecivolts = (static_cast<int32_t>(frame[3]) << 8) | frame[4];
    }
}

void receiveCrsfFrames() {
    static uint8_t frame[CRSF_FRAME_MAX];
    static uint8_t fill = 0;
    static uint32_t lastByteMillis = 0;

    if (fill != 0 && millis() - lastByteMillis > RX_IDLE_RESYNC_MS) {
        fill = 0;  // The rest never arrived, so hunt for a sync byte again.
    }

    while (crsfUart.available() > 0) {
        const uint8_t byte = static_cast<uint8_t>(crsfUart.read());
        lastByteMillis = millis();

        if (fill == 0) {
            if (isCrsfSyncByte(byte)) {
                frame[fill++] = byte;
            }
            continue;
        }

        if (fill == 1) {
            if (byte >= CRSF_LEN_MIN && byte <= CRSF_LEN_MAX) {
                frame[fill++] = byte;
            } else if (isCrsfSyncByte(byte)) {
                frame[0] = byte;  // Implausible length, so restart from here.
            } else {
                fill = 0;
            }
            continue;
        }

        frame[fill++] = byte;
        if (fill == static_cast<uint8_t>(frame[1] + 2)) {
            handleReceivedFrame(frame);
            fill = 0;
        }
    }
}

// The RC stream has to keep running while LVGL redraws, which can block for
// tens of milliseconds, so the link lives on the other core.
void crsfLinkTask(void *) {
    TickType_t wakeTime = xTaskGetTickCount();
    uint8_t pollsSinceFrame = 0;
    for (;;) {
        receiveCrsfFrames();
        if (++pollsSinceFrame >= POLLS_PER_FRAME) {
            pollsSinceFrame = 0;
            sendCrsfFrame();
        }
        xTaskDelayUntil(&wakeTime, pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

// ============================================================= LVGL glue ====

void flushDisplay(lv_disp_drv_t *display, const lv_area_t *area, lv_color_t *pixels) {
    const int32_t width = area->x2 - area->x1 + 1;
    const int32_t height = area->y2 - area->y1 + 1;
    M5.Display.startWrite();
    M5.Display.setAddrWindow(area->x1, area->y1, width, height);
    M5.Display.writePixels(reinterpret_cast<lgfx::rgb565_t *>(pixels), width * height);
    M5.Display.endWrite();
    lv_disp_flush_ready(display);
}

void readTouch(lv_indev_drv_t *, lv_indev_data_t *data) {
    data->state = LV_INDEV_STATE_REL;
    if (M5.Touch.getCount() == 0) {
        return;
    }
    const auto touch = M5.Touch.getDetail(0);
    // The Core2 digitizer extends below the panel, where it drives BtnA/B/C.
    if (!touch.isPressed() || touch.y >= SCREEN_H) {
        return;
    }
    data->point.x = touch.x;
    data->point.y = touch.y;
    data->state = LV_INDEV_STATE_PR;
}

// ============================================================== UI logic ====

// The wire carries 1000-2000 microseconds, but a transmitter is read as full
// deflection either side of centre, so that is what the readouts show. The
// halved span is added before dividing to round to the nearest percent rather
// than always truncating towards zero.
void formatChannelValue(char *text, size_t size, uint16_t value) {
    constexpr int32_t HALF_SPAN = CHANNEL_CENTER - CHANNEL_MIN;
    const int32_t scaled = (static_cast<int32_t>(value) - CHANNEL_CENTER) * 100;
    const int32_t percent =
        (scaled + (scaled >= 0 ? HALF_SPAN / 2 : -HALF_SPAN / 2)) / HALF_SPAN;
    if (percent == 0) {
        snprintf(text, size, "0%%");  // "+0%" would be noise
    } else {
        snprintf(text, size, "%+d%%", static_cast<int>(percent));
    }
}

// A channel sitting away from its default is tinted, so one glance shows
// everything that has been moved. Channels at rest keep whatever
// colour their position in the layout calls for.
uint32_t channelTint(uint8_t channel, uint32_t base) {
    return channelValues[channel] == CHANNELS[channel].defaultValue ? base
                                                                    : COLOR_CHANGED;
}

void setChannelValue(uint8_t slot, int32_t value) {
    const uint8_t channel = channelForSlot(slot);
    channelValues[channel] = static_cast<uint16_t>(value);
    char text[12];
    formatChannelValue(text, sizeof(text), static_cast<uint16_t>(value));
    lv_label_set_text(rows[slot].value, text);
    lv_obj_set_style_text_color(rows[slot].value,
                                lv_color_hex(channelTint(channel, COLOR_TEXT)), 0);
}

void sliderChanged(lv_event_t *event) {
    const uint8_t slot = static_cast<uint8_t>(
        reinterpret_cast<uintptr_t>(lv_event_get_user_data(event)));
    setChannelValue(slot, lv_slider_get_value(rows[slot].control));
}

void switchChanged(lv_event_t *event) {
    const uint8_t slot = static_cast<uint8_t>(
        reinterpret_cast<uintptr_t>(lv_event_get_user_data(event)));
    const ChannelConfig &config = CHANNELS[channelForSlot(slot)];
    const uint16_t selected = static_cast<uint16_t>(
        lv_btnmatrix_get_selected_btn(rows[slot].control));
    if (selected >= config.positions) {
        return;  // LV_BTNMATRIX_BTN_NONE, which the matrix reports on release
    }
    setChannelValue(slot, switchPositionValue(config, static_cast<uint8_t>(selected)));
}

void resetSlot(uint8_t slot) {
    const ChannelConfig &config = CHANNELS[channelForSlot(slot)];
    if (config.control == CONTROL_SWITCH) {
        lv_btnmatrix_set_btn_ctrl(rows[slot].control,
                                  switchIndexForValue(config, config.defaultValue),
                                  LV_BTNMATRIX_CTRL_CHECKED);
    } else {
        lv_slider_set_value(rows[slot].control, config.defaultValue, LV_ANIM_OFF);
    }
    setChannelValue(slot, config.defaultValue);
}

// Tapping the value readout returns that one channel to its default.
void resetChannel(lv_event_t *event) {
    resetSlot(static_cast<uint8_t>(
        reinterpret_cast<uintptr_t>(lv_event_get_user_data(event))));
}

// Every channel goes back to the value it powers on with, which for a switch
// is a detent rather than the midpoint a plain centre would have forced.
void resetAllChannels() {
    for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
        channelValues[channel] = CHANNELS[channel].defaultValue;
    }
    for (uint8_t slot = 0; slot < ROW_COUNT; ++slot) {
        resetSlot(slot);
    }
}

void resetAllClicked(lv_event_t *) { resetAllChannels(); }

void updateStatus() {
    const uint32_t lastTelemetry = lastTelemetryMillis;
    const bool linked = lastTelemetry != 0 && millis() - lastTelemetry < LINK_TIMEOUT_MS;
    const int32_t decivolts = batteryDecivolts;

    char text[24];
    if (!linked) {
        snprintf(text, sizeof(text), "NO FC");
    } else if (decivolts >= 0) {
        snprintf(text, sizeof(text), "FC OK  %d.%dV", static_cast<int>(decivolts / 10),
                 static_cast<int>(decivolts % 10));
    } else {
        snprintf(text, sizeof(text), "FC OK");
    }

    static char lastText[24] = {};
    if (strcmp(text, lastText) != 0) {
        lv_label_set_text(statusLabel, text);
        lv_obj_set_style_text_color(
            statusLabel, lv_color_hex(linked ? COLOR_LINK_OK : COLOR_TEXT_DIM), 0);
        strncpy(lastText, text, sizeof(lastText) - 1);
        lastText[sizeof(lastText) - 1] = 0;
    }
}

// ============================================================== UI build ====

lv_obj_t *makeFooterButton(lv_obj_t *parent, lv_coord_t x, const char *caption,
                           lv_event_cb_t callback) {
    lv_obj_t *button = lv_btn_create(parent);
    lv_obj_set_size(button, BUTTON_W, BUTTON_H);
    lv_obj_align(button, LV_ALIGN_LEFT_MID, x, 0);
    lv_obj_set_style_pad_all(button, 0, 0);  // so the caption centres on the button
    lv_obj_set_style_radius(button, 5, 0);
    lv_obj_set_style_bg_color(button, lv_color_hex(COLOR_BUTTON), 0);
    lv_obj_set_style_shadow_width(button, 0, 0);
    lv_obj_add_event_cb(button, callback, LV_EVENT_CLICKED, nullptr);

    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, caption);
    lv_obj_set_style_text_color(label, lv_color_hex(COLOR_TEXT), 0);
    lv_obj_center(label);
    return button;
}

void makeChannelRow(lv_obj_t *parent, uint8_t slot, uint8_t channel) {
    void *slotData = reinterpret_cast<void *>(static_cast<uintptr_t>(slot));
    const ChannelConfig &config = CHANNELS[channel];
    rows[slot].channel = channel;

    lv_obj_t *row = lv_obj_create(parent);
    lv_obj_set_size(row, ROW_W, ROW_H);
    lv_obj_set_pos(row, 0, slot * (ROW_H + ROW_GAP));
    lv_obj_set_style_pad_all(row, 0, 0);
    lv_obj_set_style_border_width(row, 0, 0);
    lv_obj_set_style_radius(row, 4, 0);
    lv_obj_set_style_bg_color(row, lv_color_hex(COLOR_ROW), 0);
    lv_obj_set_style_text_font(row, &lv_font_montserrat_14, 0);
    lv_obj_clear_flag(row, LV_OBJ_FLAG_SCROLLABLE);

    // The name has the top line to itself, so it no longer shares a 50 px
    // column with the control and "Vid Channel" fits without truncation.
    lv_obj_t *name = lv_label_create(row);
    lv_obj_set_width(name, NAME_W);
    lv_label_set_long_mode(name, LV_LABEL_LONG_DOT);
    char caption[16];
    if (config.name[0] == 0) {
        snprintf(caption, sizeof(caption), "CH %u", channel + 1);
    } else {
        snprintf(caption, sizeof(caption), "%s", config.name);
    }
    lv_label_set_text(name, caption);
    lv_obj_set_style_text_color(name, lv_color_hex(COLOR_TEXT), 0);
    lv_obj_align(name, LV_ALIGN_TOP_LEFT, NAME_X, NAME_Y);

    const uint16_t current = channelValues[channel];

    if (config.control == CONTROL_SWITCH) {
        lv_obj_t *matrix = lv_btnmatrix_create(row);
        lv_btnmatrix_set_map(matrix, switchMap(config.positions));
        lv_btnmatrix_set_one_checked(matrix, true);
        lv_obj_set_size(matrix, SWITCH_W, SWITCH_H);
        lv_obj_align(matrix, LV_ALIGN_TOP_LEFT, SWITCH_X, SWITCH_Y);
        lv_obj_set_style_pad_all(matrix, 0, 0);
        lv_obj_set_style_pad_column(matrix, 2, 0);
        lv_obj_set_style_pad_row(matrix, 0, 0);
        lv_obj_set_style_border_width(matrix, 0, 0);
        lv_obj_set_style_bg_opa(matrix, LV_OPA_TRANSP, LV_PART_MAIN);
        lv_obj_set_style_bg_color(matrix, lv_color_hex(COLOR_BUTTON), LV_PART_ITEMS);
        lv_obj_set_style_text_color(matrix, lv_color_hex(COLOR_TEXT_DIM), LV_PART_ITEMS);
        lv_obj_set_style_radius(matrix, 3, LV_PART_ITEMS);
        lv_obj_set_style_text_font(matrix, &lv_font_montserrat_12, LV_PART_ITEMS);
        // LV_PART_* and LV_STATE_* are separate unnamed enums, so combining
        // them directly is a deprecated enum-to-enum bitwise operation.
        constexpr lv_style_selector_t CHECKED_ITEM =
            static_cast<lv_style_selector_t>(LV_PART_ITEMS) |
            static_cast<lv_style_selector_t>(LV_STATE_CHECKED);
        lv_obj_set_style_bg_color(matrix, lv_color_hex(COLOR_INDICATOR), CHECKED_ITEM);
        lv_obj_set_style_text_color(matrix, lv_color_hex(COLOR_BG), CHECKED_ITEM);
        for (uint8_t index = 0; index < config.positions; ++index) {
            lv_btnmatrix_set_btn_ctrl(matrix, index, LV_BTNMATRIX_CTRL_CHECKABLE);
        }
        lv_btnmatrix_set_btn_ctrl(matrix, switchIndexForValue(config, current),
                                  LV_BTNMATRIX_CTRL_CHECKED);
        lv_obj_add_event_cb(matrix, switchChanged, LV_EVENT_VALUE_CHANGED, slotData);
        rows[slot].control = matrix;
    } else {
        lv_obj_t *slider = lv_slider_create(row);
        lv_slider_set_range(slider, CHANNEL_MIN, CHANNEL_MAX);
        lv_slider_set_value(slider, current, LV_ANIM_OFF);
        lv_obj_set_size(slider, SLIDER_W, SLIDER_H);
        lv_obj_align(slider, LV_ALIGN_TOP_LEFT, SLIDER_X, SLIDER_Y);
        lv_obj_set_style_bg_color(slider, lv_color_hex(COLOR_TRACK), LV_PART_MAIN);
        lv_obj_set_style_bg_color(slider, lv_color_hex(COLOR_INDICATOR), LV_PART_INDICATOR);
        lv_obj_set_style_bg_color(slider, lv_color_hex(COLOR_KNOB), LV_PART_KNOB);
        // The track stays thin, but the knob overhangs it and the click area is
        // grown so the lower band of the row all drives the slider.
        lv_obj_set_style_pad_all(slider, 4, LV_PART_KNOB);
        lv_obj_set_ext_click_area(slider, 4);
        lv_obj_add_event_cb(slider, sliderChanged, LV_EVENT_VALUE_CHANGED, slotData);
        rows[slot].control = slider;
    }

    lv_obj_t *value = lv_label_create(row);
    lv_obj_set_width(value, VALUE_W);
    lv_obj_set_style_text_align(value, LV_TEXT_ALIGN_RIGHT, 0);
    char text[12];
    formatChannelValue(text, sizeof(text), current);
    lv_label_set_text(value, text);
    lv_obj_set_style_text_color(value, lv_color_hex(channelTint(channel, COLOR_TEXT)), 0);
    lv_obj_align(value, LV_ALIGN_TOP_RIGHT, -6, NAME_Y);
    lv_obj_add_flag(value, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(value, resetChannel, LV_EVENT_CLICKED, slotData);
    rows[slot].value = value;
}

void createUi() {
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(COLOR_BG), 0);
    lv_obj_set_style_text_color(screen, lv_color_hex(COLOR_TEXT), 0);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *header = lv_obj_create(screen);
    lv_obj_set_size(header, SCREEN_W, HEADER_H);
    lv_obj_set_pos(header, 0, 0);
    lv_obj_set_style_pad_all(header, 0, 0);
    lv_obj_set_style_radius(header, 0, 0);
    lv_obj_set_style_border_width(header, 0, 0);
    lv_obj_set_style_bg_color(header, lv_color_hex(COLOR_PANEL), 0);
    lv_obj_clear_flag(header, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(header);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(COLOR_TEXT), 0);
    lv_label_set_text(title, "CRSF TESTER");
    lv_obj_align(title, LV_ALIGN_LEFT_MID, 6, 0);

    statusLabel = lv_label_create(header);
    lv_obj_set_style_text_font(statusLabel, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(statusLabel, lv_color_hex(COLOR_TEXT_DIM), 0);
    lv_label_set_text(statusLabel, "NO FC");
    lv_obj_align(statusLabel, LV_ALIGN_RIGHT_MID, -6, 0);

    lv_obj_t *list = lv_obj_create(screen);
    lv_obj_set_size(list, SCREEN_W, LIST_HEIGHT);
    lv_obj_set_pos(list, 0, HEADER_H);
    lv_obj_set_style_pad_all(list, 0, 0);
    lv_obj_set_style_pad_top(list, 2, 0);
    lv_obj_set_style_pad_left(list, SCREEN_GUTTER, 0);
    lv_obj_set_style_radius(list, 0, 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, 0);
    lv_obj_clear_flag(list, LV_OBJ_FLAG_SCROLLABLE);
    for (uint8_t slot = 0; slot < ROW_COUNT; ++slot) {
        makeChannelRow(list, slot, PANEL_CHANNELS[slot]);
    }

    lv_obj_t *footer = lv_obj_create(screen);
    lv_obj_set_size(footer, SCREEN_W, FOOTER_H);
    lv_obj_set_pos(footer, 0, SCREEN_H - FOOTER_H);
    lv_obj_set_style_pad_all(footer, 0, 0);
    lv_obj_set_style_radius(footer, 0, 0);
    lv_obj_set_style_border_width(footer, 0, 0);
    lv_obj_set_style_bg_color(footer, lv_color_hex(COLOR_PANEL), 0);
    lv_obj_clear_flag(footer, LV_OBJ_FLAG_SCROLLABLE);

    makeFooterButton(footer, RESET_BUTTON_X, "RESET", resetAllClicked);
}

}  // namespace

void setup() {
    auto config = M5.config();
    config.internal_spk = false;
    config.internal_mic = false;
    M5.begin(config);  // config.clear_display defaults to true
    M5.Display.setRotation(1);
    M5.Display.setBrightness(180);

    for (uint8_t channel = 0; channel < CHANNEL_COUNT; ++channel) {
        channelValues[channel] = CHANNELS[channel].defaultValue;
    }

    crsfUart.setRxBufferSize(CRSF_RX_BUFFER);
    crsfUart.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);

    lv_init();
    lv_disp_draw_buf_init(&drawBuffer, drawPixels, nullptr,
                          SCREEN_W * DRAW_BUFFER_LINES);
    lv_disp_drv_init(&displayDriver);
    displayDriver.hor_res = SCREEN_W;
    displayDriver.ver_res = SCREEN_H;
    displayDriver.flush_cb = flushDisplay;
    displayDriver.draw_buf = &drawBuffer;
    lv_disp_drv_register(&displayDriver);

    lv_indev_drv_init(&touchDriver);
    touchDriver.type = LV_INDEV_TYPE_POINTER;
    touchDriver.read_cb = readTouch;
    lv_indev_drv_register(&touchDriver);

    createUi();

    // Core 1 runs the Arduino loop and LVGL, so the link gets core 0 to itself.
    xTaskCreatePinnedToCore(crsfLinkTask, "crsf", 4096, nullptr, 3, nullptr, 0);
}

void loop() {
    M5.update();
    if (M5.BtnB.wasClicked()) {
        resetAllChannels();
    }
    updateStatus();
    lv_timer_handler();
    delay(5);
}
