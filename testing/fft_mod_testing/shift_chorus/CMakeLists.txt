# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(REPO_PATH "C:/Users/Gabriel/esp32-voice-mod")
set(COMPONENTS_PATH "${REPO_PATH}/components")
list(APPEND EXTRA_COMPONENT_DIRS ${COMPONENTS_PATH})
project(shift_chorus)