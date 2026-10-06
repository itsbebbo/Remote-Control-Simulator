# Crossfire Tester for M5Stack Core2

A PlatformIO/Arduino tester UI for the M5Stack Core2. It transmits CRSF
`RC_CHANNELS_PACKED` frames at 50 Hz over Port A and decodes the telemetry the
flight controller sends back.

> Bench tool. Remove propellers before connecting it to a flight controller.
> Every channel powers on at its own default: ARM and FIRE low, throttle at
> idle. Nothing here prevents you from switching ARM on.

## Hardware

- M5Stack Core2
- Port A pin 32 (SDA) is the UART TX. Wire it to the flight controller's CRSF RX pad.
- Port A pin 33 (SCL) is the UART RX. Wire it to the flight controller's CRSF TX pad.
- Share a ground between the two boards.
- CRSF runs at 420000 baud, 8N1, not inverted.

The pin constants sit at the top of `src/main.cpp` so they can be changed for a
different adapter or wiring arrangement.

## Controls

One screen with four controls:

| Channel | Control | Power-on value |
| --- | --- | --- |
| CH7 ARM | Two-position switch (LO / HI) | LO (1000) |
| CH8 FIRE | Two-position switch (LO / HI) | LO (1000) |
| CH9 Cam Servo | Three-position switch (LO / MID / HI) | HI (2000) |
| CH11 Contrast | Slider, 1000-2000 | Centre (1500) |

All 16 channels go out in every frame, because `RC_CHANNELS_PACKED` always
carries 16. The channels without a control stay at their defaults: the sticks
centred and the throttle at idle.

| Action | How |
| --- | --- |
| Change a channel | Tap a switch position or drag the slider |
| Reset one channel | Tap that channel's value readout |
| Reset every channel | `RESET`, or the Core2 BtnB bezel button |

Readouts show percent deflection (`-100%` to `+100%`). A channel away from its
default is shown in amber.

Channel names, control types and defaults are set in the `CHANNELS` table in
`src/main.cpp`. The `PANEL_CHANNELS` list next to it chooses which of them
appear on screen.

## Telemetry

The header shows the link state on the right:

- `NO FC` — no CRC-valid CRSF frame received in the last second.
- `FC OK` — frames are arriving, but no battery telemetry yet.
- `FC OK 12.4V` — battery sensor frames (`0x08`) are being decoded.

The receiver accepts the standard CRSF sync bytes, validates the length byte and
checks the CRC, and resynchronises if a frame stops part way through.

## How it is put together

The CRSF link runs in its own FreeRTOS task pinned to core 0. LVGL redraws can
block for tens of milliseconds, and sharing a loop with them would stretch the
gaps between RC frames far enough to risk a failsafe on the flight controller.
Core 1 runs the Arduino loop and the UI; the two share only the channel array
and two telemetry values, each a single aligned word.

## Build and upload

1. Install PlatformIO for VS Code.
2. Open this folder in VS Code.
3. Run PlatformIO Build, then Upload.

`platformio.ini` pins the pioarduino ESP32 platform, which ships arduino-esp32
3.3.x. Board support comes from **M5Unified**, not the older M5Core2 library —
M5Core2 0.2.x only builds against arduino-esp32 2.x. If you would rather stay on
M5Core2, pin `platform = espressif32@^6.11.0` instead and revert the display,
touch and button calls in `src/main.cpp` to the `M5.Lcd` / `M5.Touch.getPressPoint()`
API.

LVGL is configured by `include/lv_conf.h`, which only overrides the options that
differ from the LVGL defaults. `LV_TICK_CUSTOM` is what feeds LVGL its time base;
without it no timer fires and the screen stays blank.
