#include "usque.hpp"
#include "gtest/gtest.h" //Must include this for tests to work


//An example test.
// https://google.github.io/googletest/primer.html
TEST(Example, test) {
	//Expect Eq
	Eigen::Vector4d zeroVec = Eigen::Vector4d::Zero();
	Eigen::Matrix3d matrix = Usque::attitudeMatrix(zeroVec);
	Eigen::Matrix3d zeroMatrix = Eigen::Matrix3d::Zero();
	EXPECT_EQ(matrix, zeroMatrix);
}