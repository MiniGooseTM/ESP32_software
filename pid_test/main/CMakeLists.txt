# The IMU, motor and log drivers and the hardware config (pins, IMU axes,
# safety limits, WiFi) are shared with the robot firmware, so a fix there
# applies here too.
set(ROBOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../robot/main")

idf_component_register(SRCS "main.c"
                            "balance.c"
                            "net.c"
                            "${ROBOT_DIR}/imu.c"
                            "${ROBOT_DIR}/motors.c"
                            "${ROBOT_DIR}/log_stream.c"
                       INCLUDE_DIRS "." "${ROBOT_DIR}"
                       REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_pcnt esp_driver_i2c
                                esp_driver_gptimer esp_timer esp_hw_support esp_ringbuf
                                esp_wifi esp_netif esp_event nvs_flash esp_http_server
                       EMBED_TXTFILES "web/index.html")

# Loop rate for this test; also picks the IMU output data rate in config.h
target_compile_definitions(${COMPONENT_LIB} PRIVATE CONTROL_HZ=200)
