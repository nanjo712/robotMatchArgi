cmake_minimum_required(VERSION 3.24)



set(
        MotorLib_SRC
        #BASE
        #    ${CMAKE_CURRENT_SOURCE_DIR}/dji_board_can.c
        #    ${CMAKE_CURRENT_SOURCE_DIR}/dji_board_can.h
        ${CMAKE_CURRENT_SOURCE_DIR}/dji_boardv2_can.c
        ${CMAKE_CURRENT_SOURCE_DIR}/dji_boardv2_can.h
        ${CMAKE_CURRENT_SOURCE_DIR}/motor_driver.c
        ${CMAKE_CURRENT_SOURCE_DIR}/motor_driver.h
        ${CMAKE_CURRENT_SOURCE_DIR}/odrive_can.c
        ${CMAKE_CURRENT_SOURCE_DIR}/odrive_can.h
        ${CMAKE_CURRENT_SOURCE_DIR}/vesc_can.c
        ${CMAKE_CURRENT_SOURCE_DIR}/vesc_can.h
)

set(MotorLib ${MotorLib_SRC} PARENT_SCOPE)
set(
        MotorLib_INC PARENT_SCOPE
        ${CMAKE_CURRENT_SOURCE_DIR}
)
