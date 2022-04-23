#include "usque.hpp"

Eigen::Matrix<double, 3, 2 * __N__ + 1> sigmaOmegas(
	Eigen::Vector3d& gyroMeas, 
	//Eigen::VectorXd& error, 
	Eigen::Matrix<double, __N__, 2 * __N__ + 1>& chi
) {
	// cerr << "entered sigmaOmegas" << endl;
	Eigen::Matrix<double, 3, 2 * __N__ + 1> possAngV;
	//Note: bottom 3 rows are the errors.
	possAngV << gyroMeas.replicate<1,2 * __N__ + 1>() - chi.bottomRows(3);
	return possAngV;
}