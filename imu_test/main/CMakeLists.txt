idf_component_register(SRCS "imu_test_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c esp_timer)
