# Generate a test for a board
# TEST the test application, implemented in ${TEST}.cpp
# BOARD_LIB a library for a board containing SystemInit() and a linker script for embedded platforms
function(board_test TEST BOARD_LIB LED_TYPE)
    # check if board library exists for the current platform
    if(TARGET ${BOARD_LIB})
        string(REGEX REPLACE ".*\\:" "" BOARD ${BOARD_LIB})
        set(NAME "${TEST}-${BOARD}-${LED_TYPE}")
        message("*** Test ${TEST} on board ${BOARD} with LEDs of type ${LED_TYPE}")

        add_executable(${NAME}
            ${TEST}.cpp
        )
        target_compile_definitions(${NAME}
            PRIVATE
                ${LED_TYPE}
        )
        target_include_directories(${NAME}
            PRIVATE
                ../
                ${BOARD}
        )
        target_link_libraries(${NAME}
            ${BOARD_LIB}
            ${PROJECT_NAME}
        )

        # generate hex file for flashing the target
        if(${CMAKE_CROSSCOMPILING})
            #message("*** Generate Hex for ${NAME} using ${CMAKE_OBJCOPY}")
            add_custom_command(TARGET ${NAME}
                POST_BUILD
                COMMAND ${CMAKE_OBJCOPY} -O ihex ${NAME} ${NAME}.hex
            )
        endif()
    endif()
endfunction()

# LedControl-g431 board
if(${PLATFORM} STREQUAL "stm32g431xx")
    add_library(LedControl-g431
        LedControl-g431/config.hpp
        LedControl-g431/debug.cpp
        LedControl-g431/SystemInit.cpp
    )
    target_include_directories(LedControl-g431
        PUBLIC
            LedControl-g431
    )
    target_link_libraries(LedControl-g431
        coco::coco
    )

    #set_target_properties(${NAME} PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${BOARD}/link.ld)
    target_link_options(LedControl-g431
        PUBLIC
            "-T${CMAKE_CURRENT_SOURCE_DIR}/LedControl-g431/link.ld"
    )
endif()

board_test(LedStripTest coco-devboards::native RGB)
board_test(LedStripTest coco-devboards::emu RGB)
board_test(LedStripTest coco-devboards::nrf52dongle WS2812)
board_test(LedStripTest coco-devboards::nucleo-c031c6 WS2812)
board_test(LedStripTest coco-devboards::stm32f3348discovery WS2812)
board_test(LedStripTest coco-devboards::nucleo-g431rb WS2812)
board_test(LedStripTest coco-devboards::nucleo-g474re WS2812)
board_test(LedStripTest coco-devboards::nucleo-h503rb WS2812)

board_test(LedStripTest LedControl-g431 WS2812)
