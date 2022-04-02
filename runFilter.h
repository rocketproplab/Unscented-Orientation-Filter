#include "lib/Eigen/Cholesky"
#include "lib/Eigen/LU"


void runFilter(
	Eigen::Vector4d& attitudeQuat, 
	Eigen::MatrixXd& covariance, 
	Eigen::Vector3d& gyroBias
);