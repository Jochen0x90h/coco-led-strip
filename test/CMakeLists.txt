# Generate a test for a board
# TEST the test application, implemented in ${TEST}.cpp
# BOARD_LIB a library for a board containing SystemInit() and a linker script for embedded platforms
function(board_test TEST BOARD_LIB)
	# check if board library exists for the current platform
	if(TARGET ${BOARD_LIB})
		string(REGEX REPLACE ".*\\:" "" BOARD ${BOARD_LIB})
		set(NAME "${TEST}-${BOARD}")
		message("*** Test ${TEST} on board ${BOARD}")

		add_executable(${NAME}
			${TEST}.cpp
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

# progbox board, only testing
if(${PLATFORM} STREQUAL "stm32g474xx")
	add_library(progbox
		progbox/config.hpp
		progbox/debug.cpp
		progbox/SystemInit.cpp
	)
	target_include_directories(progbox
		PUBLIC
		progbox
	)
	target_link_libraries(progbox
		coco::coco
	)

	#set_target_properties(${NAME} PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${BOARD}/link.ld)
	target_link_options(progbox
		PUBLIC
			"-T${CMAKE_CURRENT_SOURCE_DIR}/progbox/link.ld"
	)
endif()

board_test(LedStripTest coco-devboards::native)
board_test(LedStripTest coco-devboards::emu)
board_test(LedStripTest coco-devboards::nrf52dongle)
board_test(LedStripTest coco-devboards::stm32f3348discovery)
board_test(LedStripTest coco-devboards::stm32c031nucleo)
board_test(LedStripTest coco-devboards::stm32g431nucleo)
board_test(LedStripTest coco-devboards::stm32g474nucleo)

board_test(LedStripTest progbox)
