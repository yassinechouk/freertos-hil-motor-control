idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_mcpwm esp_driver_pcnt esp_timer)