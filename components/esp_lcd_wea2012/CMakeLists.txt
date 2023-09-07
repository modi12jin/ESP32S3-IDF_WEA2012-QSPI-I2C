idf_component_register(SRCS "esp_lcd_panel_io_qspi.c" "esp_lcd_wea2012.c"
                    INCLUDE_DIRS "include"
                    PRIV_REQUIRES driver
                    REQUIRES esp_lcd)
