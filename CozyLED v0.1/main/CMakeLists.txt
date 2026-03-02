idf_component_register(SRCS "CozyLED.c"
                    PRIV_REQUIRES spi_flash
                    REQUIRES driver esp_timer ssd1306
                    INCLUDE_DIRS "")
