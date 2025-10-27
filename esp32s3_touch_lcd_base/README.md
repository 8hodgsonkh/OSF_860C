# ESP32-S3 Touch LCD 7" Base Project

This ESP-IDF v6.0 project targets the Waveshare ESP32-S3-Touch-LCD-7 development kit (800×480 RGB panel with FT5x06 touch). It boots, enables PSRAM-based double buffering, initializes the LCD using the on-chip RGB peripheral, and brings up an LVGL UI with touch input.

## Hardware Mapping

| Function | ESP32-S3 GPIO |
|----------|---------------|
| LCD HSYNC | 46 |
| LCD VSYNC | 3 |
| LCD DE | 5 |
| LCD PCLK | 7 |
| LCD Data[0..15] | 14, 38, 18, 17, 10, 39, 0, 45, 48, 47, 21, 1, 2, 42, 41, 40 |
| Backlight (via CH422 EXIO2) | 2* |
| Touch SDA | 8 |
| Touch SCL | 9 |
| Touch IRQ | 4 |
| Touch RST | EXIO1 (handled in firmware) |

\*The board routes the backlight gate through the CH422 I/O expander. The helper in `main.c` toggles it over I²C; if your hardware revision wires it straight to GPIO2, the same function keeps the pin high. Adjust `lcd_backlight_set()` if you relocate the signal.

## Building

```bash
cd esp32s3_touch_lcd_base
idf.py set-target esp32s3
idf.py build
```

Flash over the USB download port:

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

## Required menuconfig Settings

`idf.py menuconfig` will preload the values from `sdkconfig.defaults`. Verify or adjust the following:

- **Serial flasher config → Flash SPI mode**: QIO @ 120 MHz.
- **Serial flasher config → Flash size**: 16 MB.
- **Component config → ESP System Settings → Default CPU frequency**: 240 MHz.
- **Component config → ESP PSRAM**:
  - Enable Octal PSRAM, 120 MHz, initialize during boot, allow malloc from PSRAM.
  - Set “Reserve internal memory” ≥ 32 KB to keep fast heap for ISR usage.
- **Component config → LCD RGB**:
  - Enable “Use PSRAM for frame buffers” and “Put ISR handlers into IRAM”.
  - Pixel clock source: PLL160M (auto when 120 MHz flash/PSRAM is selected).
- **Component config → LVGL configuration**:
  - Color depth RGB565, custom tick (5 ms), task priority ≥ 4, LVGL memory ≥ 128 KB.
- **Component config → Driver Configuration → I2C**: default 400 kHz master (I2C0 on GPIO8/9).

After the first build, the generated `sdkconfig` will contain all dependencies needed by LVGL, the RGB LCD, PSRAM, and the FT5x06 touch driver.

## UI Layout

- Black background.
- Centered white label that reads **“CONTROL ESTABLISHED.”**
- A status label in the lower-left corner that shows CPU frequency and free heap; it updates once per second.

Touch input is configured through LVGL and can be inspected in the serial log. The template code contains comments outlining how to add additional widgets (buttons, gauges, warning banners) for future e-bike telemetry pages.

## Notes

- The LVGL draw buffers live entirely in PSRAM (double buffering) for smooth 60 Hz refresh.
- `sdkconfig.defaults` already opts into performance-oriented compiler flags; adjust if you need size optimization.
- All drivers are pure ESP-IDF components—no Arduino APIs.
