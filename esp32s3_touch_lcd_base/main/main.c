#include <stdio.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_touch_ft5x06.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_private/esp_clk.h"
#include "lvgl.h"

#define TAG "LCD_BASE"

#define LCD_H_RES            800
#define LCD_V_RES            480
#define LCD_BIT_DEPTH        16
#define LCD_PIXEL_CLOCK_HZ   (24 * 1000 * 1000)

#define I2C_PORT             I2C_NUM_0
#define I2C_FREQ_HZ          400000
#define I2C_TIMEOUT_TICKS    pdMS_TO_TICKS(100)

#define LCD_PIN_NUM_HSYNC    GPIO_NUM_46
#define LCD_PIN_NUM_VSYNC    GPIO_NUM_3
#define LCD_PIN_NUM_DE       GPIO_NUM_5
#define LCD_PIN_NUM_PCLK     GPIO_NUM_7

static const gpio_num_t lcd_data_pins[16] = {
    GPIO_NUM_14, GPIO_NUM_38, GPIO_NUM_18, GPIO_NUM_17,
    GPIO_NUM_10, GPIO_NUM_39, GPIO_NUM_0,  GPIO_NUM_45,
    GPIO_NUM_48, GPIO_NUM_47, GPIO_NUM_21, GPIO_NUM_1,
    GPIO_NUM_2,  GPIO_NUM_42, GPIO_NUM_41, GPIO_NUM_40,
};

#define LCD_BACKLIGHT_GPIO   GPIO_NUM_2
#define CH422_ADDR_CFG       0x24
#define CH422_ADDR_DATA      0x38

#define TOUCH_I2C_ADDRESS    0x38
#define TOUCH_INT_GPIO       GPIO_NUM_4
#define TOUCH_RST_GPIO       GPIO_NUM_NC

#define LVGL_TICK_PERIOD_MS  5
#define LVGL_TASK_DELAY_MS   10

static SemaphoreHandle_t lvgl_mutex;
static esp_timer_handle_t lvgl_tick_timer;

static esp_lcd_panel_handle_t panel_handle;
static esp_lcd_touch_handle_t touch_handle;

static void lvgl_update_status_cb(lv_timer_t *timer);

static bool lvgl_lock(uint32_t timeout_ms)
{
    if (lvgl_mutex == NULL) {
        return false;
    }
    return xSemaphoreTakeRecursive(lvgl_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

static void lvgl_unlock(void)
{
    if (lvgl_mutex) {
        xSemaphoreGiveRecursive(lvgl_mutex);
    }
}

static void lvgl_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_task(void *arg)
{
    (void)arg;
    while (true) {
        if (lvgl_lock(LVGL_TASK_DELAY_MS)) {
            lv_timer_handler();
            lvgl_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(LVGL_TASK_DELAY_MS));
    }
}

static esp_err_t i2c_master_bus_init(void)
{
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_8,
        .scl_io_num = GPIO_NUM_9,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(I2C_PORT, &i2c_conf), TAG, "I2C param config failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0), TAG, "I2C install failed");
    return ESP_OK;
}

static esp_err_t ch422_write(uint8_t addr, uint8_t data)
{
    return i2c_master_write_to_device(I2C_PORT, addr, &data, 1, I2C_TIMEOUT_TICKS);
}

static void ensure_backlight_gpio(void)
{
    static bool configured = false;
    if (!configured) {
        gpio_config_t cfg = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << LCD_BACKLIGHT_GPIO,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
        };
        gpio_config(&cfg);
        configured = true;
    }
}

static esp_err_t lcd_backlight_set(bool enable)
{
    uint8_t buf = 0x01;
    esp_err_t ret = ch422_write(CH422_ADDR_CFG, buf);
    if (ret == ESP_OK) {
        buf = enable ? 0x1E : 0x1A;
        ret = ch422_write(CH422_ADDR_DATA, buf);
    }

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "CH422 backlight control failed (%s), falling back to direct GPIO", esp_err_to_name(ret));
        ensure_backlight_gpio();
        gpio_set_level(LCD_BACKLIGHT_GPIO, enable);
        ret = ESP_OK;
    }

    return ret;
}

static void lvgl_flush_cb(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    esp_lcd_panel_handle_t handle = (esp_lcd_panel_handle_t)disp_drv->user_data;
    esp_lcd_panel_draw_bitmap(handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_p);
    lv_disp_flush_ready(disp_drv);
}

static void lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    esp_lcd_touch_handle_t touch = (esp_lcd_touch_handle_t)drv->user_data;
    uint16_t touch_x[1];
    uint16_t touch_y[1];
    uint8_t touch_cnt = 0;

    if (touch == NULL) {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    esp_lcd_touch_read_data(touch);
    bool pressed = esp_lcd_touch_get_coordinates(touch, touch_x, touch_y, NULL, &touch_cnt, 1);

    if (pressed && touch_cnt > 0) {
        data->point.x = touch_x[0];
        data->point.y = touch_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void create_ui(void)
{
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "CONTROL ESTABLISHED.");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, LV_FONT_DEFAULT, 0);
    lv_obj_center(title);

    lv_obj_t *status = lv_label_create(screen);
    lv_obj_set_style_text_color(status, lv_color_white(), 0);
    lv_label_set_text(status, "CPU 0 MHz | Heap 0 KB");
    lv_obj_align(status, LV_ALIGN_BOTTOM_LEFT, 12, -12);

    lv_timer_create(lvgl_update_status_cb, 1000, status);

    /* Future expansion ideas:
     * - Attach LVGL button widgets to trigger motor assist modes.
     * - Add an LVGL arc or gauge to show speed/current using real telemetry.
     * - Overlay warning banners or animation layers for alerts (over-temp, low battery, etc.).
     */
}

static void lvgl_update_status_cb(lv_timer_t *timer)
{
    lv_obj_t *label = (lv_obj_t *)timer->user_data;
    uint32_t cpu_freq = esp_clk_cpu_freq() / 1000000U;
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT) / 1024U;
    lv_label_set_text_fmt(label, "CPU %u MHz | Heap %u KB", cpu_freq, (unsigned)free_heap);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing I2C bus");
    ESP_ERROR_CHECK(i2c_master_bus_init());

    ESP_LOGI(TAG, "Enabling backlight rail");
    ESP_ERROR_CHECK(lcd_backlight_set(false));

    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = LCD_BIT_DEPTH,
        .bits_per_pixel = LCD_BIT_DEPTH,
        .num_fbs = 1,
        .bounce_buffer_size_px = 0,
        .sram_trans_align = 4,
        .psram_trans_align = 64,
        .hsync_gpio_num = LCD_PIN_NUM_HSYNC,
        .vsync_gpio_num = LCD_PIN_NUM_VSYNC,
        .de_gpio_num = LCD_PIN_NUM_DE,
        .pclk_gpio_num = LCD_PIN_NUM_PCLK,
        .disp_gpio_num = GPIO_NUM_NC,
        .data_gpio_nums = {
            lcd_data_pins[0],  lcd_data_pins[1],  lcd_data_pins[2],  lcd_data_pins[3],
            lcd_data_pins[4],  lcd_data_pins[5],  lcd_data_pins[6],  lcd_data_pins[7],
            lcd_data_pins[8],  lcd_data_pins[9],  lcd_data_pins[10], lcd_data_pins[11],
            lcd_data_pins[12], lcd_data_pins[13], lcd_data_pins[14], lcd_data_pins[15],
        },
        .timings = {
            .pclk_hz = LCD_PIXEL_CLOCK_HZ,
            .h_res = LCD_H_RES,
            .v_res = LCD_V_RES,
            .hsync_pulse_width = 4,
            .hsync_back_porch = 8,
            .hsync_front_porch = 8,
            .vsync_pulse_width = 4,
            .vsync_back_porch = 8,
            .vsync_front_porch = 8,
            .flags = {
                .pclk_active_neg = 1,
                .hsync_idle_high = 0,
                .vsync_idle_high = 0,
                .de_idle_high = 0,
            },
        },
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .flags = {
            .fb_in_psram = 1,
        },
    };

    ESP_LOGI(TAG, "Creating RGB panel instance");
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_LOGI(TAG, "Initializing LVGL");
    lv_init();

    lvgl_mutex = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mutex);

    static lv_disp_draw_buf_t draw_buf;
    size_t buffer_pixels = LCD_H_RES * LCD_V_RES;
    size_t buffer_size = buffer_pixels * sizeof(lv_color_t);
    // Allocate two full-screen draw buffers in PSRAM for double buffering
    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf1 || !buf2) {
        ESP_LOGE(TAG, "LVGL buffer allocation failed");
        abort();
    }

    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, buffer_pixels);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.user_data = panel_handle;
    disp_drv.full_refresh = 1;
    lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Configuring FT5x06 touch controller");
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_FT5X06_CONFIG();
    tp_io_config.dev_addr = TOUCH_I2C_ADDRESS;
    tp_io_config.scl_speed_hz = I2C_FREQ_HZ;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_PORT, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = TOUCH_RST_GPIO,
        .int_gpio_num = TOUCH_INT_GPIO,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        }, // Adjust swap/mirror here if the touch axes feel inverted
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_ft5x06(tp_io_handle, &tp_cfg, &touch_handle));

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_touch_cb;
    indev_drv.user_data = touch_handle;
    lv_indev_drv_register(&indev_drv);

    if (lvgl_lock(0)) {
        create_ui();
        lvgl_unlock();
    }

    esp_timer_create_args_t tick_args = {
        .callback = &lvgl_tick_cb,
        .name = "lv_tick",
    };
    ESP_ERROR_CHECK(esp_timer_create(&tick_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Spawning LVGL task");
    xTaskCreatePinnedToCore(lvgl_task, "lvgl", 8192, NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "Turning on LCD backlight");
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_ERROR_CHECK(lcd_backlight_set(true));

    ESP_LOGI(TAG, "Startup complete");
}
