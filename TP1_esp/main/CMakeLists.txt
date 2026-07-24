idf_component_register(SRCS "main.c" "motor_driver.c" "ultrasonic.c" "encoder.c"
                       INCLUDE_DIRS "." "include"
                       PRIV_REQUIRES micro_ros_espidf_component
                       REQUIRES driver esp_timer esp_rom)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)