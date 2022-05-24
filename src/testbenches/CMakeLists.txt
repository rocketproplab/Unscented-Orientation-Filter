add_executable(Testbench 
	testbench-sim.cpp
)

target_link_libraries(Testbench 
	PUBLIC Sim
)