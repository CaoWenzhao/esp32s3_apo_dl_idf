cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/components/board"
    "${CMAKE_CURRENT_LIST_DIR}/components/sensors/a02yyuw"
    "${CMAKE_CURRENT_LIST_DIR}/components/sensors/bu_uwb"
    "${CMAKE_CURRENT_LIST_DIR}/components/sensors/fsr_adc"
    "${CMAKE_CURRENT_LIST_DIR}/components/sensors/rplidar_c1"
    "${CMAKE_CURRENT_LIST_DIR}/components/control/chassis"
    "${CMAKE_CURRENT_LIST_DIR}/components/control/ble_control"
    "${CMAKE_CURRENT_LIST_DIR}/components/control/follow_avoid"
    "${CMAKE_CURRENT_LIST_DIR}/components/control/web_control"
)

set(COMPONENTS main)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(follow_robot)
