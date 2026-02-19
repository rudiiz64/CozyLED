idf_component_register(SRCS "CozyLED.c"
                    PRIV_REQUIRES spi_flash
                    REQUIRES driver esp_timer
                    INCLUDE_DIRS "")
