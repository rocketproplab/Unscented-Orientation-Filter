# Google Test
include(FetchContent)
FetchContent_Declare(
	googletest
	URL https://github.com/google/googletest/archive/e2239ee6043f73722e7aa812a459f54a28552929.zip
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)



enable_testing()


add_executable(playground playground.cpp)
target_link_libraries(playground gtest_main)
include(GoogleTest)
gtest_discover_tests(playground)