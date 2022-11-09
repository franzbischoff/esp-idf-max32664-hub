idf_component_register(
    SRCS
    INCLUDE_DIRS "include"
    REQUIRES "driver"
)
set_source_files_properties(
    "max32664.cpp"
    PROPERTIES COMPILE_FLAGS "-std=c++14"
)
