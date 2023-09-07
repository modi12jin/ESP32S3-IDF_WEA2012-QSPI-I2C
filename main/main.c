#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "lv_demos.h"

#include "esp_lcd_panel_io_qspi.h"
#include "esp_lcd_wea2012.h"

#include "driver/i2c.h"
#include "esp_lcd_touch_wea2012.h"

static const char *TAG = "example";

// 在示例中使用SPI2
#define LCD_HOST SPI2_HOST
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ SPI_MASTER_FREQ_80M
#define EXAMPLE_PIN_NUM_SCLK 5
#define EXAMPLE_PIN_NUM_D0 14
#define EXAMPLE_PIN_NUM_D1 8
#define EXAMPLE_PIN_NUM_D2 0
#define EXAMPLE_PIN_NUM_D3 1
#define EXAMPLE_PIN_NUM_LCD_RST 6
#define EXAMPLE_PIN_NUM_LCD_CS 10

#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL 1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_BK_LIGHT 13

#define EXAMPLE_I2C_NUM 0 // I2C number
#define EXAMPLE_I2C_SCL 9
#define EXAMPLE_I2C_SDA 3
#define EXAMPLE_PIN_NUM_TP_RST 4
#define EXAMPLE_PIN_NUM_TP_INT 11

#define EXAMPLE_LCD_H_RES 356
#define EXAMPLE_LCD_V_RES 400

// 用于表示命令和参数的位数
#define EXAMPLE_LCD_CMD_BITS 8
#define EXAMPLE_LCD_PARAM_BITS 8

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2

esp_lcd_touch_handle_t tp = NULL;

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // 将缓冲区的内容复制到显示的特定区域
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

/* 在 LVGL 中旋转屏幕时，旋转显示和触摸。 更新驱动程序参数时调用. */
static void example_lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;

    switch (drv->rotated)
    {
    case LV_DISP_ROT_NONE:
        // 旋转液晶显示屏
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, false);
        // 旋转液晶触摸
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
        break;
    case LV_DISP_ROT_90:
        // 旋转液晶显示屏
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, true);
        // 旋转液晶触摸
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
        break;
    case LV_DISP_ROT_180:
        // 旋转液晶显示屏
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, true);
        // 旋转液晶触摸
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
        break;
    case LV_DISP_ROT_270:
        // 旋转液晶显示屏
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, false);
        // 旋转液晶触摸
        esp_lcd_touch_set_mirror_y(tp, false);
        esp_lcd_touch_set_mirror_x(tp, false);
        break;
    }
}

static void example_lvgl_port_rounder_callback(struct _lv_disp_drv_t *disp_drv, lv_area_t *area)
{
    uint16_t x1 = area->x1;
    uint16_t x2 = area->x2;

    // 将区域的起始位置向下舍入到最接近的4N数字
    area->x1 = (x1 >> 2) << 2;

    // 将区域末尾四舍五入到最接近的4M+3数字
    area->x2 = ((x2 >> 2) << 2) + 3;
}

static void example_lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* 读取触摸控制器数据 */
    esp_lcd_touch_read_data(drv->user_data);

    /* 获取坐标 */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0)
    {

        ESP_LOGI(TAG, "Touch x=%d,y=%d ", touchpad_x[0], touchpad_y[0]);
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    }
    else
    {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

static void example_increase_lvgl_tick(void *arg)
{
    /* 告诉LVGL已经过去了多少毫秒 */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

void app_main(void)
{
    static lv_disp_draw_buf_t disp_buf; // 包含称为绘制缓冲区的内部图形缓冲区
    static lv_disp_drv_t disp_drv;      // 包含回调函数

    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT};
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "Initialize SPI bus");

    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .data0_io_num = EXAMPLE_PIN_NUM_D0,
        .data1_io_num = EXAMPLE_PIN_NUM_D1,
        .data2_io_num = EXAMPLE_PIN_NUM_D2,
        .data3_io_num = EXAMPLE_PIN_NUM_D3,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t),
        .flags = SPICOMMON_BUSFLAG_QUAD,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    // 将LCD连接到SPI总线

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_qspi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install WEA2012 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_wea2012(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    //  ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    // esp_lcd_panel_swap_xy(panel_handle, true);
    // esp_lcd_panel_mirror(panel_handle, true, false);

    // 在打开屏幕或背光之前，用户可以将预定义的图案刷新到屏幕上
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    ESP_LOGI(TAG, "Initialize I2C");

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = EXAMPLE_I2C_SDA,
        .scl_io_num = EXAMPLE_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    /* 初始化I2C */
    ESP_ERROR_CHECK(i2c_param_config(EXAMPLE_I2C_NUM, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(EXAMPLE_I2C_NUM, i2c_conf.mode, 0, 0, 0));

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_WEA2012_CONFIG();

    ESP_LOGI(TAG, "Initialize touch IO (I2C)");

    /* Touch IO handle */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)EXAMPLE_I2C_NUM, &tp_io_config, &tp_io_handle));

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_V_RES,
        .y_max = EXAMPLE_LCD_H_RES,
        .rst_gpio_num = EXAMPLE_PIN_NUM_TP_RST,
        .int_gpio_num = EXAMPLE_PIN_NUM_TP_INT,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };

    /* 初始化触摸 */
    ESP_LOGI(TAG, "Initialize touch controller WEA2012");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_wea2012(tp_io_handle, &tp_cfg, &tp));

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // 分配 LVGL 使用的绘制缓冲区
    // 建议选择绘制缓冲区的大小至少为屏幕大小的 1/10
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // 初始化 LVGL 绘制缓冲区
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.drv_update_cb = example_lvgl_port_update_callback;
    disp_drv.rounder_cb = example_lvgl_port_rounder_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // LVGL 的 Tick 接口（使用 esp_timer 生成 2ms 周期性事件）
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"};
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    static lv_indev_drv_t indev_drv; // 输入设备驱动程序（触摸）
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;

    lv_indev_drv_register(&indev_drv);

    ESP_LOGI(TAG, "Display LVGL Meter Widget");
    lv_demo_widgets();
    // lv_demo_benchmark(); //基准测试
    // lv_demo_music();

    while (1)
    {
        // 提高 LVGL 的任务优先级和/或减少处理程序周期可以提高性能
        vTaskDelay(pdMS_TO_TICKS(10));
        // 运行 lv_timer_handler 的任务的优先级应该低于运行 `lv_tick_inc` 的任务
        lv_timer_handler();
    }
}
