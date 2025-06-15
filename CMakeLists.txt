idf_component_register(SRCS "src/ADS1115_WE.cpp"
                    INCLUDE_DIRS "src"
                    REQUIRES esp_driver_i2c esp_timer)
