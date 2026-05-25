idf_component_register(
    SRCS
        "mcp4728.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_i2c
)
