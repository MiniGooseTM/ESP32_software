idf_component_register(SRCS "main.c"
                            "control.c"
                            "imu.c"
                            "motors.c"
                            "policy.c"
                            "net.c"
                            "log_stream.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_driver_i2c
                                esp_driver_gptimer esp_timer esp_hw_support esp_ringbuf
                                esp_wifi esp_netif esp_event nvs_flash esp_http_server
                       EMBED_TXTFILES "web/index.html")

# The policy forward pass is the hot path; optimise it for speed regardless of
# the project-wide optimisation level.
set_source_files_properties(policy.c PROPERTIES COMPILE_OPTIONS "-O3")
