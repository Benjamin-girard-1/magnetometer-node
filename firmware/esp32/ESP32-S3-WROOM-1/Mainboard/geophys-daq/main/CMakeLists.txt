idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        shift_register
        board_detect
        mcp4728
        hmc100x
        lsm6dsv
        ad7779
        scl3300
)
