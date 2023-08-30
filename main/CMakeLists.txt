idf_component_register(SRCS "MLX90640_driver.c" "main.c"
                    REQUIRES driver json
                    INCLUDE_DIRS "." "include")