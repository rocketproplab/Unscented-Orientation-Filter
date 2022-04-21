#include "Eigen/Cholesky"

extern Eigen::Matrix3d crossMatrix(Eigen::Vector3d& vec);
extern Eigen::Matrix3d crossMatrix(Eigen::Vector3d&& vec);

Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d& vecLeft,
	Eigen::Vector4d& vecRight
) {
	Eigen::Matrix4d eps;
	eps.block(0,0,3,3) << vecRight(3) * Eigen::Matrix3d::Identity() + 
		crossMatrix(1 * vecRight.head<3>());
	eps.block(3,0,1,3) << -1*vecRight.block(0,0,3,1).transpose();
	eps.col(3) << vecRight;
	Eigen::Vector4d prodVec = eps*vecLeft;
	return prodVec;
}