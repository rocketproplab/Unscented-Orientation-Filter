cmake_minimum_required(VERSION 3.10)
project(Usof VERSION 0.1) #Unscented orientation filter

add_subdirectory(lib/eigen-3.4.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# set(ENABLE_UNIT_TESTS False CACHE BOOL "Enable/disable unit tests")

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# USQUE library source
add_subdirectory(src/usque)
# Simulation source files
add_subdirectory(src/simulation)
#Unit tests
# if(ENABLE_UNIT_TESTS) {
	# add_subdirectory(tests)
# }
# Testbench
add_subdirectory(src/testbenches)