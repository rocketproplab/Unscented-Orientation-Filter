# Google Test
include(FetchContent)
FetchContent_Declare(
	googletest
	URL https://github.com/google/googletest/archive/e2239ee6043f73722e7aa812a459f54a28552929.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)



enable_testing()

add_executable(Example example.cpp)
add_executable(IdealGyroVals IdealGyroVals.cpp)
target_link_libraries(Example 
	PRIVATE gtest_main
	PUBLIC Usque
)

target_link_libraries(IdealGyroVals 
	PRIVATE gtest_main
	PUBLIC Sim
)
include(GoogleTest)
gtest_discover_tests(Example)
gtest_discover_tests(IdealGyroVals)