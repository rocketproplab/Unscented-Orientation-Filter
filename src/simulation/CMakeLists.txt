# add_library(Sim STATIC
# 	${CMAKE_CURRENT_SOURCE_DIR}/meas.cxx 
# 	${CMAKE_CURRENT_SOURCE_DIR}/utils.cxx 
# )

add_library(Sim STATIC
	meas.cpp 
	utils.cpp 
)
target_link_libraries(Sim 
	PUBLIC Eigen3::Eigen
	PUBLIC Usque
)
target_include_directories(Sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})