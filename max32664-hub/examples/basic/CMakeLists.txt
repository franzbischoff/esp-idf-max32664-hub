cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# set(ENV{IDF_TOOLCHAIN} clang) # needs to install xtensa-clang first: idf_tools.py install xtensa-clang
list(APPEND compile_options "-Wno-parentheses-equality"
  "-Wno-unused-function")
project(max32664_hub_example)
