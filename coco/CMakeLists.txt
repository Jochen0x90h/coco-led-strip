set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
add_library(${PROJECT_NAME})
#target_sources(${PROJECT_NAME}
#	PUBLIC FILE_SET headers TYPE HEADERS FILES
#		Color.hpp
#	PRIVATE
#)

if(${PLATFORM} STREQUAL "native")
	# native platform (Windows, MacOS, Linux)
	target_sources(${PROJECT_NAME}
		PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/native FILES
			native/coco/platform/LedStrip_cout.hpp
		PRIVATE
			native/coco/platform/LedStrip_cout.cpp
	)
elseif(${PLATFORM} STREQUAL "emu")
	# emulator platform with graphical user interface (Windows, MacOS, Linux)
	target_sources(${PROJECT_NAME}
		PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/emu FILES
			emu/coco/platform/GuiLedStrip.hpp
			emu/coco/platform/LedStrip_emu.hpp
		PRIVATE
			emu/coco/platform/GuiLedStrip.cpp
			emu/coco/platform/LedStrip_emu.cpp
	)
elseif(${PLATFORM} MATCHES "^nrf52")
	target_sources(${PROJECT_NAME}
		PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/nrf52 FILES
			nrf52/coco/platform/LedStrip_I2S.hpp
		PRIVATE
			nrf52/coco/platform/LedStrip_I2S.cpp
	)
elseif(${PLATFORM} MATCHES "^stm32")
	target_sources(${PROJECT_NAME}
		PUBLIC FILE_SET platform_headers TYPE HEADERS BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/stm32 FILES
			stm32/coco/platform/LedStrip_UART_DMA.hpp
		PRIVATE
			stm32/coco/platform/LedStrip_UART_DMA.cpp
	)
endif()

target_link_libraries(${PROJECT_NAME}
	coco::coco
	coco-loop::coco-loop
	coco-device::coco-device
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
		..
)

# install the library
install(TARGETS ${PROJECT_NAME}
	FILE_SET headers DESTINATION include/coco
	FILE_SET platform_headers DESTINATION include
)
