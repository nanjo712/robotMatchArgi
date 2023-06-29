cmake_minimum_required(VERSION 3.24)

set(ChassisType rubberwheel_chassis)
message("Used Chassis Type : ${ChassisType}")
set(
    ChassisLib_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/chassis_common_config.h
    ${CMAKE_CURRENT_SOURCE_DIR}/base_chassis.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/base_chassis.h
    ${CMAKE_CURRENT_SOURCE_DIR}/ctrlGo2Point.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ctrlGo2Point.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../SimpleLib/utils/utils.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../SimpleLib/utils/utils.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../SimpleLib/utils/vec.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../SimpleLib/utils/vec.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../SimpleLib/utils/kalman.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../SimpleLib/utils/kalman.h
)

if( rubberwheel_chassis IN_LIST ChassisType)
    LIST(
            APPEND ChassisLib_SRC
            ${CMAKE_CURRENT_SOURCE_DIR}/rubberwheel_chassis.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/rubberwheel_chassis.h
    )#RUBBERWHEELSRC
endif ()

set(ChassisLib ${ChassisLib_SRC} PARENT_SCOPE)
set(
    ChassisLib_INC PARENT_SCOPE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../SimpleLib/utils
)
