
#find_package(Boost COMPONENTS system filesystem)

#include_directories(${Boost_INCLUDE_DIRS})

#message("Boost_INCLUDE_DIRS ${Boost_INCLUDE_DIRS}")

add_executable(generator
	main.cpp
)

#target_link_libraries(generator
#	${Boost_LIBRARIES}
#)

#install(TARGETS generator
#	RUNTIME DESTINATION bin)
