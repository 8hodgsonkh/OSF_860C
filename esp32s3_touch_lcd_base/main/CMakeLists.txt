idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_lcd esp_lcd_touch esp_lcd_touch_ft5x06 lvgl driver esp_timer
)
