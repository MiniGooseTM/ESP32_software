idf_component_register(SRCS "main.c"
                            "control.c"
                            "imu.c"
                            "motors.c"
                            "policy.c"
                            "policy_net.c"
                            "net.c"
                            "log_stream.c"
                            "battery.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_driver_i2c
                                esp_driver_gptimer esp_adc esp_timer esp_hw_support esp_ringbuf
                                esp_wifi esp_netif esp_event nvs_flash esp_http_server esp_partition
                       EMBED_TXTFILES "web/index.html"
                       # Built-in policy, from `make export` in software_stack
                       EMBED_FILES "policy.bin")

set_source_files_properties(policy_net.c PROPERTIES COMPILE_OPTIONS "-O3")
