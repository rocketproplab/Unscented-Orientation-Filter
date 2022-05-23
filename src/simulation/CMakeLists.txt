# add_library(Sim STATIC
# 	${CMAKE_CURRENT_SOURCE_DIR}/meas.cxx 
# 	${CMAKE_CURRENT_SOURCE_DIR}/utils.cxx 
# )

add_library(Sim STATIC
	utils.cpp 
	simulation.cpp
)
target_link_libraries(Sim 
	PUBLIC Eigen3::Eigen
	PUBLIC Usque
)
# TODO: Add testing directive?
target_include_directories(Sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})