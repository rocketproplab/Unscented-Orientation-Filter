#include "usque.hpp"

Eigen::Matrix<double, 6, 3> crossCorr(
	MatrixErr6& possNewError, //6x13
	VectorNd& predError, //6
	Eigen::Matrix<double, 3, 2 * __N__ + 1>& possExpMagMeas, //3x13
	Eigen::Vector3d& predMagMeas, //3
	const int lambda
) {
	Eigen::Matrix<double, 6, 3> covSum;
	covSum << lambda * (possNewError.col(2 * __N__) - predError) * 
		(possExpMagMeas.col(2 * __N__) - predMagMeas).transpose();
	
	for(int i = 0; i < 2 * __N__; i++) {
		covSum = covSum + (possNewError.col(i) - predError) * 
			0.5*(possExpMagMeas.col(i) - predMagMeas).transpose();
	}

	return covSum/(__N__ + lambda);
}