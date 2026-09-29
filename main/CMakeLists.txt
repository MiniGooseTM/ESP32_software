idf_component_register(SRCS "motor_test_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt)
