#include "usque.hpp"

Eigen::MatrixXd innovationCov(
	Eigen::MatrixXd possExpMagMeas, 
	Eigen::Vector3d predMagMeas, 
	int lambda, 
	double sigma_mag
) {
	const int n = 6;
	MatrixXd covSum;
	covSum << lambda*(possExpMagMeas.col(2*n) - predMagMeas) * 
		(possExpMagMeas.col(2*n) - predMagMeas).transpose();
	
	for(int i = 0; i < 2*n; i++) {
		covSum = covSum + (possExpMagMeas.col(i) - predMagMeas) * 
			0.5*(possExpMagMeas.col(i) - predMagMeas).transpose();
	}

	return covSum/(n + lambda) + (sigma_mag*sigma_mag) * 
		MatrixXd::Identity(3,3);
}
Eigen::MatrixXd innovationCov(
	Eigen::Matrix<double, 3, 2 * __N__ + 1>& possExpMagMeas, 
	Eigen::Vector3d& predMagMeas, 
	int lambda, 
	double sigma_mag
) {

}