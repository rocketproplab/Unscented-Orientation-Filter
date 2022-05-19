# Usque library
add_library(Usque STATIC 
	usque.cpp 
)
target_link_libraries(Usque PUBLIC Eigen3::Eigen)
target_include_directories(Usque PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

