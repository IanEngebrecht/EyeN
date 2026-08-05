cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(EyeN)

idf_build_set_property(CXX_COMPILE_OPTIONS "-std=gnu++26" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wall;-Wextra;-Werror" APPEND)
