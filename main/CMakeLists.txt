
set(INCLUDES "mlx90640/Include/mlx90640-driver/" ".")
    file(GLOB_RECURSE SOURCES "mlx90640/Source/mlx90640-driver/*.c" "*.c")

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS ${INCLUDES}
                    )
