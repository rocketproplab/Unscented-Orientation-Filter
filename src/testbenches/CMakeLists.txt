add_executable(testbench 
	${CMAKE_CURRENT_SOURCE_DIR}/usque.cxx 
)

target_link_libraries(testbench Usof Meas)