idf_component_register(
    SRCS "max32664.cpp"
    INCLUDE_DIRS "include"
    REQUIRES "driver"
)

set_source_files_properties(
    "max32664.cpp"
    PROPERTIES COMPILE_FLAGS "--std=gnu++11"
)
