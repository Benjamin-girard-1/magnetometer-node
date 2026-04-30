idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES shift_register
    REQUIRES lsm6dsv
    REQUIRES ad7779
    REQUIRES scl3300
)