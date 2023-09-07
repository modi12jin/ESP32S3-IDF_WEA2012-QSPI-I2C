#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_wea2012.h"

static const char *TAG = "wea2012";

static esp_err_t panel_wea2012_del(esp_lcd_panel_t *panel);
static esp_err_t panel_wea2012_reset(esp_lcd_panel_t *panel);
static esp_err_t panel_wea2012_init(esp_lcd_panel_t *panel);
static esp_err_t panel_wea2012_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data);
static esp_err_t panel_wea2012_invert_color(esp_lcd_panel_t *panel, bool invert_color_data);
static esp_err_t panel_wea2012_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y);
static esp_err_t panel_wea2012_swap_xy(esp_lcd_panel_t *panel, bool swap_axes);
static esp_err_t panel_wea2012_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap);
static esp_err_t panel_wea2012_disp_on_off(esp_lcd_panel_t *panel, bool off);

typedef struct
{
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    unsigned int bits_per_pixel;
    uint8_t madctl_val; // save current value of LCD_CMD_MADCTL register
    uint8_t colmod_cal; // save surrent value of LCD_CMD_COLMOD register
} wea2012_panel_t;

esp_err_t esp_lcd_new_panel_wea2012(const esp_lcd_panel_io_handle_t io, const esp_lcd_panel_dev_config_t *panel_dev_config, esp_lcd_panel_handle_t *ret_panel)
{
    esp_err_t ret = ESP_OK;
    wea2012_panel_t *wea2012 = NULL;
    ESP_GOTO_ON_FALSE(io && panel_dev_config && ret_panel, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    wea2012 = calloc(1, sizeof(wea2012_panel_t));
    ESP_GOTO_ON_FALSE(wea2012, ESP_ERR_NO_MEM, err, TAG, "no mem for wea2012 panel");

    if (panel_dev_config->reset_gpio_num >= 0)
    {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << panel_dev_config->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "configure GPIO for RST line failed");
    }

    switch (panel_dev_config->color_space)
    {
    case ESP_LCD_COLOR_SPACE_RGB:
        wea2012->madctl_val = 0;
        break;
    case ESP_LCD_COLOR_SPACE_BGR:
        wea2012->madctl_val |= LCD_CMD_BGR_BIT;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color space");
        break;
    }

    switch (panel_dev_config->bits_per_pixel)
    {
    case 16:
        wea2012->colmod_cal = 0x05;
        break;
    case 18:
        wea2012->colmod_cal = 0x06;
        break;
    case 24:
        wea2012->colmod_cal = 0x07;
        break;
    default:
        ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
        break;
    }

    wea2012->io = io;
    wea2012->bits_per_pixel = panel_dev_config->bits_per_pixel;
    wea2012->reset_gpio_num = panel_dev_config->reset_gpio_num;
    wea2012->reset_level = panel_dev_config->flags.reset_active_high;
    wea2012->base.del = panel_wea2012_del;
    wea2012->base.reset = panel_wea2012_reset;
    wea2012->base.init = panel_wea2012_init;
    wea2012->base.draw_bitmap = panel_wea2012_draw_bitmap;
    wea2012->base.invert_color = panel_wea2012_invert_color;
    wea2012->base.set_gap = panel_wea2012_set_gap;
    wea2012->base.mirror = panel_wea2012_mirror;
    wea2012->base.swap_xy = panel_wea2012_swap_xy;
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    wea2012->base.disp_off = panel_wea2012_disp_on_off;
#else
    wea2012->base.disp_on_off = panel_wea2012_disp_on_off;
#endif
    *ret_panel = &(wea2012->base);
    ESP_LOGI(TAG, "new wea2012 panel @%p", wea2012);

    return ESP_OK;

err:
    if (wea2012)
    {
        if (panel_dev_config->reset_gpio_num >= 0)
        {
            gpio_reset_pin(panel_dev_config->reset_gpio_num);
        }
        free(wea2012);
    }
    return ret;
}

static esp_err_t panel_wea2012_del(esp_lcd_panel_t *panel)
{
    wea2012_panel_t *wea2012 = __containerof(panel, wea2012_panel_t, base);

    if (wea2012->reset_gpio_num >= 0)
    {
        gpio_reset_pin(wea2012->reset_gpio_num);
    }
    ESP_LOGD(TAG, "del wea2012 panel @%p", wea2012);
    free(wea2012);
    return ESP_OK;
}

static esp_err_t panel_wea2012_reset(esp_lcd_panel_t *panel)
{
    wea2012_panel_t *wea2012 = __containerof(panel, wea2012_panel_t, base);
    esp_lcd_panel_io_handle_t io = wea2012->io;

    // perform hardware reset
    if (wea2012->reset_gpio_num >= 0)
    {
        gpio_set_level(wea2012->reset_gpio_num, wea2012->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(wea2012->reset_gpio_num, !wea2012->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    else
    { // perform software reset
        esp_lcd_panel_io_tx_param(io, LCD_CMD_SWRESET, NULL, 0);
        vTaskDelay(pdMS_TO_TICKS(20)); // spec, wait at least 5ms before sending new command
    }

    return ESP_OK;
}

typedef struct
{
    uint8_t cmd;
    uint8_t data[4];
    uint8_t len;
} lcd_cmd_data_t;

static const lcd_cmd_data_t lcd_cmd_data[] = {
    //==============================
    //=Start of SPD2010 Initial Code
    //==============================
    {0xFF, {0x20, 0x10, 0x10}, 3},
    {0x0C, {0x11}, 1},
    {0x10, {0x02}, 1},
    {0x11, {0x11}, 1},
    {0x15, {0x42}, 1},
    {0x16, {0x11}, 1},
    {0x1A, {0x02}, 1},
    {0x1B, {0x11}, 1},
    {0x61, {0x80}, 1},
    {0x62, {0x80}, 1},
    {0x54, {0x44}, 1},
    {0x58, {0x88}, 1},
    {0x5C, {0xcc}, 1},
    {0xFF, {0x20, 0x10, 0x10}, 3},
    {0x20, {0x80}, 1},
    {0x21, {0x81}, 1},
    {0x22, {0x31}, 1},
    {0x23, {0x20}, 1},
    {0x24, {0x11}, 1},
    {0x25, {0x11}, 1},
    {0x26, {0x12}, 1},
    {0x27, {0x12}, 1},
    {0x30, {0x80}, 1},
    {0x31, {0x81}, 1},
    {0x32, {0x31}, 1},
    {0x33, {0x20}, 1},
    {0x34, {0x11}, 1},
    {0x35, {0x11}, 1},
    {0x36, {0x12}, 1},
    {0x37, {0x12}, 1},
    {0xFF, {0x20, 0x10, 0x10}, 3},
    {0x41, {0x11}, 1},
    {0x42, {0x22}, 1},
    {0x43, {0x33}, 1},
    {0x49, {0x11}, 1},
    {0x4A, {0x22}, 1},
    {0x4B, {0x33}, 1},
    {0xFF, {0x20, 0x10, 0x15}, 3},
    {0x00, {0x00}, 1},
    {0x01, {0x00}, 1},
    {0x02, {0x00}, 1},
    {0x03, {0x00}, 1},

    //{0x04, {0x08}, 1},
    //{0x05, {0x04}, 1},
    //{0x06, {0x19}, 1},
    //{0x07, {0x18}, 1},
    //{0x08, {0x17}, 1},
    //{0x09, {0x16}, 1},

    {0x04, {0x10}, 1},
    {0x05, {0x0C}, 1},
    {0x06, {0x23}, 1},
    {0x07, {0x22}, 1},
    {0x08, {0x21}, 1},
    {0x09, {0x20}, 1},

    {0x0A, {0x33}, 1},
    {0x0B, {0x32}, 1},
    {0x0C, {0x34}, 1},
    {0x0D, {0x35}, 1},
    {0x0E, {0x01}, 1},
    {0x0F, {0x01}, 1},
    {0x20, {0x00}, 1},
    {0x21, {0x00}, 1},
    {0x22, {0x00}, 1},
    {0x23, {0x00}, 1},

    //{0x24, {0x04}, 1},
    //{0x25, {0x08}, 1},
    //{0x26, {0x16}, 1},
    //{0x27, {0x17}, 1},
    //{0x28, {0x18}, 1},
    //{0x29, {0x19}, 1},

    {0x24, {0x0C}, 1},
    {0x25, {0x10}, 1},
    {0x26, {0x20}, 1},
    {0x27, {0x21}, 1},
    {0x28, {0x22}, 1},
    {0x29, {0x23}, 1},

    {0x2A, {0x33}, 1},
    {0x2B, {0x32}, 1},
    {0x2C, {0x34}, 1},
    {0x2D, {0x35}, 1},
    {0x2E, {0x01}, 1},
    {0x2F, {0x01}, 1},
    {0xFF, {0x20, 0x10, 0x16}, 3},
    {0x00, {0x00}, 1},
    {0x01, {0x00}, 1},
    {0x02, {0x00}, 1},
    {0x03, {0x00}, 1},

    //{0x04, {0x10}, 1},
    //{0x05, {0x0C}, 1},
    //{0x06, {0x23}, 1},
    //{0x07, {0x22}, 1},
    //{0x08, {0x21}, 1},
    //{0x09, {0x20}, 1},

    {0x04, {0x08}, 1},
    {0x05, {0x04}, 1},
    {0x06, {0x19}, 1},
    {0x07, {0x18}, 1},
    {0x08, {0x17}, 1},
    {0x09, {0x16}, 1},

    {0x0A, {0x33}, 1},
    {0x0B, {0x32}, 1},
    {0x0C, {0x34}, 1},
    {0x0D, {0x35}, 1},
    {0x0E, {0x01}, 1},
    {0x0F, {0x01}, 1},
    {0x20, {0x00}, 1},
    {0x21, {0x00}, 1},
    {0x22, {0x00}, 1},
    {0x23, {0x00}, 1},

    //{0x24, {0x0C}, 1},
    //{0x25, {0x10}, 1},
    //{0x26, {0x20}, 1},
    //{0x27, {0x21}, 1},
    //{0x28, {0x22}, 1},
    //{0x29, {0x23}, 1},

    {0x24, {0x04}, 1},
    {0x25, {0x08}, 1},
    {0x26, {0x16}, 1},
    {0x27, {0x17}, 1},
    {0x28, {0x18}, 1},
    {0x29, {0x19}, 1},

    {0x2A, {0x33}, 1},
    {0x2B, {0x32}, 1},
    {0x2C, {0x34}, 1},
    {0x2D, {0x35}, 1},
    {0x2E, {0x01}, 1},
    {0x2F, {0x01}, 1},
    //{0xFF, {0x20,0x10,0x17}, 3},
    //{0x32, {0x00}, 1},
    {0xFF, {0x20, 0x10, 0x12}, 3},
    {0x00, {0x99}, 1},
    {0x2A, {0x28}, 1},
    {0x2B, {0x0f}, 1},
    {0x2C, {0x16}, 1},
    {0x2D, {0x28}, 1},
    {0x2E, {0x0f}, 1},
    {0xFF, {0x20, 0x10, 0xA0}, 3},
    {0x08, {0xdc}, 1},
    {0xFF, {0x20, 0x10, 0x45}, 3},
    {0x03, {0x64}, 1},
    {0xFF, {0x20, 0x10, 0x42}, 3},
    {0x05, {0x2c}, 1},
    {0xFF, {0x20, 0x10, 0x11}, 3},
    {0x50, {0x01}, 1},
    {0xFF, {0x20, 0x10, 0x00}, 3},
    {0x2A, {0x00, 0x00, 0x01, 0x63}, 4},

    // Sot error
    {0xFF, {0x20, 0x10, 0x40}, 3},
    {0x86, {0x00}, 1},
    {0xFF, {0x20, 0x10, 0x00}, 3},

    // LVD VCI=2.4V
    {0xFF, {0x20, 0x10, 0x12}, 3},
    {0x0D, {0x66}, 1},

    // GPWR period
    {0xFF, {0x20, 0x10, 0x17}, 3},
    {0x39, {0x3c}, 1},

    // Gamma 2.2

    //{0xff, {0x20,0x10,0x30}, 3},
    //{0x00, {0x15}, 1},
    //{0xff, {0x20,0x10,0x00}, 3},

    // Positive gamma
    {0xff, {0x20, 0x10, 0x31}, 3},

    // level 0
    {0x38, {0x03}, 1},
    {0x39, {0xf0}, 1},

    // level 1
    {0x36, {0x03}, 1},
    {0x37, {0xe8}, 1},

    // level 3
    {0x34, {0x03}, 1},
    //{0x35, {0xda}, 1},
    {0x35, {0xCF}, 1},

    // level 5
    {0x32, {0x03}, 1},
    //{0x33, {0xcf}, 1},
    {0x33, {0xBA}, 1},

    // level 7
    {0x30, {0x03}, 1},
    //{0x31, {0xc8}, 1},
    {0x31, {0xA2}, 1},

    // level 9
    {0x2e, {0x03}, 1},
    //{0x2f, {0xba}, 1},
    {0x2f, {0x8E}, 1},

    // level 11
    {0x2c, {0x03}, 1},
    //{0x2d, {0xac}, 1},
    {0x2d, {0x70}, 1},

    // level 13
    {0x2a, {0x03}, 1},
    //{0x2b, {0xa0}, 1},
    {0x2b, {0x52}, 1},

    // level 15
    {0x28, {0x03}, 1},
    //{0x29, {0x70}, 1},
    {0x29, {0x3E}, 1},

    // level 24
    {0x26, {0x02}, 1},
    {0x27, {0xfa}, 1},

    // level 32
    {0x24, {0x02}, 1},
    {0x25, {0xc2}, 1},

    // level 48
    {0x22, {0x02}, 1},
    {0x23, {0x80}, 1},

    // level 64
    {0x20, {0x02}, 1},
    {0x21, {0x53}, 1},

    // level 96
    {0x1e, {0x02}, 1},
    {0x1f, {0x0e}, 1},

    // level 128
    {0x1c, {0x01}, 1},
    {0x1d, {0xcf}, 1},

    // level 160
    {0x1a, {0x01}, 1},
    {0x1b, {0x7b}, 1},

    // level 192
    {0x18, {0x01}, 1},
    {0x19, {0x3d}, 1},

    // level 208
    {0x16, {0x01}, 1},
    {0x17, {0x0e}, 1},

    // level 224
    {0x14, {0x00}, 1},
    {0x15, {0xd0}, 1},

    // level 232
    {0x12, {0x00}, 1},
    {0x13, {0xa6}, 1},

    // level 240
    {0x10, {0x00}, 1},
    {0x11, {0x79}, 1},

    // level 242
    {0x0e, {0x00}, 1},
    {0x0f, {0x72}, 1},

    // level 244
    {0x0c, {0x00}, 1},
    {0x0d, {0x62}, 1},

    // level 246
    {0x0a, {0x00}, 1},
    {0x0b, {0x54}, 1},

    // level 248
    {0x08, {0x00}, 1},
    {0x09, {0x3d}, 1},

    // level 250
    {0x06, {0x00}, 1},
    {0x07, {0x36}, 1},

    // level 252
    {0x04, {0x00}, 1},
    {0x05, {0x1c}, 1},

    // level 254
    {0x02, {0x00}, 1},
    {0x03, {0x09}, 1},

    {0xff, {0x20, 0x10, 0x00}, 3},

    // negative gamma
    {0xff, {0x20, 0x10, 0x32}, 3},

    // level 0
    {0x38, {0x03}, 1},
    {0x39, {0xf0}, 1},

    // level 1
    {0x36, {0x03}, 1},
    {0x37, {0xe8}, 1},

    // level 3
    {0x34, {0x03}, 1},
    //{0x35, {0xda}, 1},
    {0x35, {0xCF}, 1},

    // level 5
    {0x32, {0x03}, 1},
    //{0x33, {0xcf}, 1},
    {0x33, {0xBA}, 1},

    // level 7
    {0x30, {0x03}, 1},
    //{0x31, {0xc8}, 1},
    {0x31, {0xA2}, 1},

    // level 9
    {0x2e, {0x03}, 1},
    //{0x2f, {0xba}, 1},
    {0x2f, {0x8E}, 1},

    // level 11
    {0x2c, {0x03}, 1},
    //{0x2d, {0xac}, 1},
    {0x2d, {0x70}, 1},

    // level 13
    {0x2a, {0x03}, 1},
    //{0x2b, {0xa0}, 1},
    {0x2b, {0x52}, 1},

    // level 15
    {0x28, {0x03}, 1},
    //{0x29, {0x70}, 1},
    {0x29, {0x3E}, 1},

    // level 24
    {0x26, {0x02}, 1},
    {0x27, {0xfa}, 1},

    // level 32
    {0x24, {0x02}, 1},
    {0x25, {0xc2}, 1},

    // level 48
    {0x22, {0x02}, 1},
    {0x23, {0x80}, 1},

    // level 64
    {0x20, {0x02}, 1},
    {0x21, {0x53}, 1},

    // level 96
    {0x1e, {0x02}, 1},
    {0x1f, {0x0e}, 1},

    // level 128
    {0x1c, {0x01}, 1},
    {0x1d, {0xcf}, 1},

    // level 160
    {0x1a, {0x01}, 1},
    {0x1b, {0x7b}, 1},

    // level 192
    {0x18, {0x01}, 1},
    {0x19, {0x3d}, 1},

    // level 208
    {0x16, {0x01}, 1},
    {0x17, {0x0e}, 1},

    // level 224
    {0x14, {0x00}, 1},
    {0x15, {0xd0}, 1},

    // level 232
    {0x12, {0x00}, 1},
    {0x13, {0xa6}, 1},

    // level 240
    {0x10, {0x00}, 1},
    {0x11, {0x79}, 1},

    // level 242
    {0x0e, {0x00}, 1},
    {0x0f, {0x72}, 1},

    // level 244
    {0x0c, {0x00}, 1},
    {0x0d, {0x62}, 1},

    // level 246
    {0x0a, {0x00}, 1},
    {0x0b, {0x54}, 1},

    // level 248
    {0x08, {0x00}, 1},
    {0x09, {0x3d}, 1},

    // level 250
    {0x06, {0x00}, 1},
    {0x07, {0x36}, 1},

    // level 252
    {0x04, {0x00}, 1},
    {0x05, {0x1c}, 1},

    // level 254
    {0x02, {0x00}, 1},
    {0x03, {0x09}, 1},

    {0xff, {0x20, 0x10, 0x00}, 3},

    //==============================

    // display 60Hz
    {0xFF, {0x20, 0x10, 0x11}, 3},
    {0x60, {0x01}, 1},
    {0x65, {0x03}, 1},
    {0x66, {0x38}, 1},
    {0x67, {0x04}, 1},
    {0x68, {0x34}, 1},
    {0x69, {0x03}, 1},
    {0x61, {0x03}, 1},
    {0x62, {0x38}, 1},
    {0x63, {0x04}, 1},
    {0x64, {0x34}, 1},
    {0x0A, {0x11}, 1},
    {0x0B, {0x14}, 1},
    {0x0c, {0x14}, 1},
    {0x55, {0x06}, 1},
    {0xFF, {0x20, 0x10, 0x42}, 3},
    {0x05, {0x3D}, 1},
    {0x06, {0x03}, 1},
    {0xFF, {0x20, 0x10, 0x00}, 3},

    // ver 0.2
    // GVDDN setting
    {0xFF, {0x20, 0x10, 0x12}, 3},
    {0x1F, {0xDC}, 1},
    // LPM
    {0xff, {0x20, 0x10, 0x17}, 3},
    {0x11, {0xAA}, 1},
    {0x16, {0x12}, 1},
    {0x0B, {0xC3}, 1},
    {0x10, {0x0E}, 1},
    {0x14, {0xAA}, 1},
    {0x18, {0xA0}, 1},
    {0x1A, {0x80}, 1},
    {0x1F, {0x80}, 1},
    {0xff, {0x20, 0x10, 0x11}, 3},
    {0x30, {0xEE}, 1},
    {0xff, {0x20, 0x10, 0x12}, 3},
    {0x15, {0x0F}, 1},

    // Internal SRAM timing
    {0xff, {0x20, 0x10, 0x2D}, 3},
    {0x01, {0x3E}, 1},

    // MIPI settling timing
    {0xff, {0x20, 0x10, 0x40}, 3},
    {0x83, {0xC4}, 1},

    // VGLO= -11, VGLRATIO changed from 4x to 5x
    {0xFF, {0x20, 0x10, 0x12}, 3},
    {0x2B, {0x1e}, 1},
    {0x2C, {0x26}, 1},
    {0x2E, {0x1e}, 1},

    // Enable all ESDDET_SET[3:0]
    {0xFF, {0x20, 0x10, 0x12}, 3},
    {0x10, {0x0F}, 1},

    // Tune source EQ
    {0xFF, {0x20, 0x10, 0x18}, 3},
    {0x01, {0x01}, 1},
    {0x00, {0x1E}, 1},

    // Disable load OTP after sleep out
    {0xFF, {0x20, 0x10, 0x43}, 3},
    {0x03, {0x04}, 1},

    //==============================
    //--	TIC Setting
    //==============================
    //--	GPIO_C [7:0]
    //--	page 0x50, cmd 0x05, data 0x00  (data bit4=0 for I2C, bit4=1 for SPI)
    {0xFF, {0x20, 0x10, 0x50}, 3},
    {0x05, {0x00}, 1},
    {0xFF, {0x20, 0x10, 0x00}, 3},

    //--	DIC_I2C_SA [7:0]
    //--	page 0x50, cmd 0x00, data 0x00    for I2CS_DA[15: 8]
    //--	page 0x50, cmd 0x01, data 0x00    for I2CS_DA[ 7: 0]
    {0xFF, {0x20, 0x10, 0x50}, 3},
    {0x00, {0xA6}, 1},
    {0x01, {0xA6}, 1},
    {0xFF, {0x20, 0x10, 0x00}, 3},

    //--	TIC_SPI_IOMUX [7:0]
    //--	page 0x50, cmd 0x08, data 0x00    [7:6] MISO, [5:4]MOSI, [3:2]SCL, [1:0] CSX
    {0xFF, {0x20, 0x10, 0x50}, 3},
    {0x08, {0x55}, 1},
    {0xFF, {0x20, 0x10, 0x00}, 3},

    // ver 0.2 end

    // Case 1, 3H
    {0xFF, {0x20, 0x10, 0x10}, 3},
    {0x0B, {0x43}, 1},
    {0x0C, {0x12}, 1},
    {0x10, {0x01}, 1},
    {0x11, {0x12}, 1},
    {0x15, {0x00}, 1},
    {0x16, {0x00}, 1},
    {0x1A, {0x00}, 1},
    {0x1B, {0x00}, 1},
    {0x61, {0x00}, 1},
    {0x62, {0x00}, 1},
    {0x51, {0x11}, 1},
    {0x55, {0x55}, 1},
    {0x58, {0x00}, 1},
    {0x5C, {0x00}, 1},

    {0xFF, {0x20, 0x10, 0x10}, 3},
    {0x20, {0x81}, 1},
    {0x21, {0x82}, 1},
    {0x22, {0x72}, 1},
    {0x30, {0x00}, 1},
    {0x31, {0x00}, 1},
    {0x32, {0x00}, 1},

    {0xFF, {0x20, 0x10, 0x10}, 3},
    {0x44, {0x44}, 1},
    {0x45, {0x55}, 1},
    {0x46, {0x66}, 1},
    {0x47, {0x77}, 1},
    {0x49, {0x00}, 1},
    {0x4A, {0x00}, 1},
    {0x4B, {0x00}, 1},

    {0xFF, {0x20, 0x10, 0x17}, 3},
    {0x37, {0x00}, 1},

    {0xFF, {0x20, 0x10, 0x15}, 3},
    {0x04, {0x08}, 1},
    {0x05, {0x04}, 1},
    {0x06, {0x1C}, 1},
    {0x07, {0x1A}, 1},
    {0x08, {0x18}, 1},
    {0x09, {0x16}, 1},

    {0x24, {0x05}, 1},
    {0x25, {0x09}, 1},
    {0x26, {0x17}, 1},
    {0x27, {0x19}, 1},
    {0x28, {0x1B}, 1},
    {0x29, {0x1D}, 1},

    {0xFF, {0x20, 0x10, 0x16}, 3},
    {0x04, {0x09}, 1},
    {0x05, {0x05}, 1},
    {0x06, {0x1D}, 1},
    {0x07, {0x1B}, 1},
    {0x08, {0x19}, 1},
    {0x09, {0x17}, 1},

    {0x24, {0x04}, 1},
    {0x25, {0x08}, 1},
    {0x26, {0x16}, 1},
    {0x27, {0x18}, 1},
    {0x28, {0x1A}, 1},
    {0x29, {0x1C}, 1},

    {0xFF, {0x20, 0x10, 0x18}, 3},
    {0x1F, {0x00}, 1},

    // case 1, 3H code end

    {0xFF, {0x20, 0x10, 0x18}, 3},
    {0x3A, {0x01}, 1},

    // Idle mode and Skip mode CP clk setting
    {0xFF, {0x20, 0x10, 0x11}, 3},

    //{0x1A, {0x00}, 1},
    //{0x1B, {0x00}, 1},

    // Idle mode
    {0x15, {0x99}, 1},
    {0x16, {0x99}, 1},
    {0x1C, {0x88}, 1},
    {0x1D, {0x88}, 1},
    {0x1E, {0x88}, 1},

    // skip
    {0x13, {0xf0}, 1},
    {0x14, {0x34}, 1},

    {0xFF, {0x20, 0x10, 0x12}, 3},
    // OSC
    // {0x12, {0x8b}, 1},
    // v0.3a
    {0x06, {0x06}, 1},
    {0x12, {0x89}, 1},
    // ####
    {0xFF, {0x20, 0x10, 0x11}, 3},
    {0x0A, {0x00}, 1},
    {0x0B, {0xf2}, 1},
    {0x0c, {0xf2}, 1},
    {0xFF, {0x20, 0x10, 0x00}, 3},
    {0xFF, {0x20, 0x10, 0x11}, 3},
    {0x08, {0x70}, 1},
    {0x09, {0x00}, 1},

    {0xFF, {0x20, 0x10, 0x00}, 3},
    {0x35, {0x00}, 1},
    {0X3A, {0X05}, 1},

    //==============================
    //=End of SPD2010 Initial Code
    //==============================

    // panel #21 VCOM
    // may need to be tuned for different panel
    {0xFF, {0x20, 0x10, 0x12}, 3},
    {0x21, {0xC8}, 1},
    //{0x21, {0x1A}, 1},

    {0xFF, {0x20, 0x10, 0x00}, 3},
    {0x11, {0x00}, 0},
    {0x29, {0x00}, 0},

    {0x35, {0x00}, 0}, // 该命令打开TE信号线的撕裂效果输出信号。

    {0x00, {0x00}, 0xff}, // 100ms

};

static esp_err_t panel_wea2012_init(esp_lcd_panel_t *panel)
{
    wea2012_panel_t *wea2012 = __containerof(panel, wea2012_panel_t, base);
    esp_lcd_panel_io_handle_t io = wea2012->io;

    // LCD进入睡眠模式，上电复位后显示屏将关闭，请先退出睡眠模式
    //  esp_lcd_panel_io_tx_param(io, LCD_CMD_SLPOUT, NULL, 0);
    //  vTaskDelay(pdMS_TO_TICKS(100));
    //  esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]) {
    //       wea2012->madctl_val,
    //   }, 1);
    //   esp_lcd_panel_io_tx_param(io, LCD_CMD_COLMOD, (uint8_t[]) {
    //       wea2012->colmod_cal,
    //   }, 1);

    // 厂商特定的初始化，不同厂商可以不同
    // 应向 LCD 供应商咨询初始化序列代码
    // 初始化屏幕
    const lcd_cmd_data_t *lcd_init = lcd_cmd_data;
    for (int i = 0; i < (sizeof(lcd_cmd_data) / sizeof(lcd_cmd_data_t)); i++)
    {
        if (lcd_init[i].len == 0xff)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else
        {
            esp_lcd_panel_io_tx_param(io, lcd_init[i].cmd,
                                      lcd_init[i].data,
                                      lcd_init[i].len);
        }
    }
    ESP_LOGI(TAG, " esp_lcd_wea2012_init");

    return ESP_OK;
}

static esp_err_t panel_wea2012_draw_bitmap(esp_lcd_panel_t *panel, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    wea2012_panel_t *wea2012 = __containerof(panel, wea2012_panel_t, base);
    assert((x_start < x_end) && (y_start < y_end) && "start position must be smaller than end position");
    esp_lcd_panel_io_handle_t io = wea2012->io;

    x_start += wea2012->x_gap;
    x_end += wea2012->x_gap;
    y_start += wea2012->y_gap;
    y_end += wea2012->y_gap;

    // define an area of frame memory where MCU can access
    esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]){
                                                     (x_start >> 8) & 0xFF,
                                                     x_start & 0xFF,
                                                     ((x_end - 1) >> 8) & 0xFF,
                                                     (x_end - 1) & 0xFF,
                                                 },
                              4);
    esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]){
                                                     (y_start >> 8) & 0xFF,
                                                     y_start & 0xFF,
                                                     ((y_end - 1) >> 8) & 0xFF,
                                                     (y_end - 1) & 0xFF,
                                                 },
                              4);
    // transfer frame buffer
    size_t len = (x_end - x_start) * (y_end - y_start) * wea2012->bits_per_pixel / 8;
    esp_lcd_panel_io_tx_color(io, LCD_CMD_RAMWR, color_data, len);

    return ESP_OK;
}

static esp_err_t panel_wea2012_invert_color(esp_lcd_panel_t *panel, bool invert_color_data)
{
    wea2012_panel_t *wea2012 = __containerof(panel, wea2012_panel_t, base);
    esp_lcd_panel_io_handle_t io = wea2012->io;
    int command = 0;
    if (invert_color_data)
    {
        command = LCD_CMD_INVON;
    }
    else
    {
        command = LCD_CMD_INVOFF;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    return ESP_OK;
}

static esp_err_t panel_wea2012_mirror(esp_lcd_panel_t *panel, bool mirror_x, bool mirror_y)
{
    wea2012_panel_t *wea2012 = __containerof(panel, wea2012_panel_t, base);
    esp_lcd_panel_io_handle_t io = wea2012->io;
    if (mirror_x)
    {
        wea2012->madctl_val |= LCD_CMD_MX_BIT;
    }
    else
    {
        wea2012->madctl_val &= ~LCD_CMD_MX_BIT;
    }
    if (mirror_y)
    {
        wea2012->madctl_val |= LCD_CMD_MY_BIT;
    }
    else
    {
        wea2012->madctl_val &= ~LCD_CMD_MY_BIT;
    }
    esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]){wea2012->madctl_val}, 1);
    return ESP_OK;
}

static esp_err_t panel_wea2012_swap_xy(esp_lcd_panel_t *panel, bool swap_axes)
{
    wea2012_panel_t *wea2012 = __containerof(panel, wea2012_panel_t, base);
    esp_lcd_panel_io_handle_t io = wea2012->io;
    if (swap_axes)
    {
        wea2012->madctl_val |= LCD_CMD_MV_BIT;
    }
    else
    {
        wea2012->madctl_val &= ~LCD_CMD_MV_BIT;
    }
    esp_lcd_panel_io_tx_param(io, LCD_CMD_MADCTL, (uint8_t[]){wea2012->madctl_val}, 1);
    return ESP_OK;
}

static esp_err_t panel_wea2012_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    wea2012_panel_t *wea2012 = __containerof(panel, wea2012_panel_t, base);
    wea2012->x_gap = x_gap;
    wea2012->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_wea2012_disp_on_off(esp_lcd_panel_t *panel, bool on_off)
{
    wea2012_panel_t *wea2012 = __containerof(panel, wea2012_panel_t, base);
    esp_lcd_panel_io_handle_t io = wea2012->io;
    int command = 0;

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    on_off = !on_off;
#endif

    if (on_off)
    {
        command = LCD_CMD_DISPON;
    }
    else
    {
        command = LCD_CMD_DISPOFF;
    }
    esp_lcd_panel_io_tx_param(io, command, NULL, 0);
    return ESP_OK;
}
