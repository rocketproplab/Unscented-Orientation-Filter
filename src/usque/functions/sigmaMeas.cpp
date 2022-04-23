#include "usque.hpp"

Eigen::Matrix<double, 3, 2 * __N__ + 1> sigmaMeas(
	Eigen::Matrix4d& possNewQuats, 
	Vector3d& magField 
) {
	Eigen::Matrix<double, 3, 2 * __N__ + 1> possExpMagMeas;
	for(int i = 0; i < 2 * __N__ + 1; i++) {
		possExpMagMeas.col(i) = attitudeMatrix(possNewQuats.col(i)) * magField;
	}
	return possExpMagMeas;
}