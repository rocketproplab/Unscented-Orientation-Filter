#include "usque.hpp"

Eigen::Matrix3d innovationCov(
    Eigen::Matrix<double, 3, 2 * SIZE + 1>& possExpMagMeas, 
	Eigen::Vector3d& predMagMeas,
    int lambda,
    double sigma_mag) {
	Eigen::Matrix3d covSum;
	covSum << lambda*(possExpMagMeas.col(2 * SIZE) - predMagMeas) * 
		(possExpMagMeas.col(2 * SIZE) - predMagMeas).transpose();
	
	for(int i = 0; i < 2*SIZE; i++) {
		covSum = covSum + (possExpMagMeas.col(i) - predMagMeas) * 
			0.5*(possExpMagMeas.col(i) - predMagMeas).transpose();
	}

	return covSum / (SIZE + lambda) + (sigma_mag*sigma_mag) * 
		Eigen::Matrix3d::Identity();
}

