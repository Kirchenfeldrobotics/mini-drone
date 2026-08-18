idf_component_register(
    SRCS "dshot.cpp"
    INCLUDE_DIRS "include"
    REQUIRES driver pid communications flight_types utils dshot_esc_encoder
)