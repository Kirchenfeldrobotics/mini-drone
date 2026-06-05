idf_component_register(
    SRCS "pid.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio flight_types mpu6050 communications
)