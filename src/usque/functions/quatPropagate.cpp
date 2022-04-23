#include "Eigen/Cholesky"

#define __N__ 6


Eigen::Matrix3d crossMatrix(Eigen::Vector3d& vec);

Eigen::Matrix3d crossMatrix(Eigen::Vector3d&& vec);

Eigen::Matrix<double, 4, 2 * __N__ + 1> quatPropagate(
	Eigen::Matrix<double, 4, 2 * __N__ + 1>& possQuats, 
	Eigen::Matrix<double, 3, 2 * __N__ + 1>& possAngV, 
	double gyroDt
) {
	Eigen::Matrix<double, 4, 2 * __N__ + 1> possNewQuats;
	for(int i = 0; i < 2 * __N__ + 1; i++) {
		Eigen::Vector3d psiK = sin(0.5*gyroDt*possAngV.col(i).norm())*
			possAngV.col(i)/possAngV.col(i).norm();
		Eigen::Matrix4d omega;
		omega.block(0,0,3,3) << cos(0.5*gyroDt*possAngV.col(i).norm())*
			Eigen::Matrix3d::Identity() - crossMatrix(psiK);
		omega.block(0,3,3,1) << psiK;
		omega.row(3) << -1*psiK.transpose(), cos(0.5*gyroDt*possAngV.col(i).
			norm());
		possNewQuats.col(i) << omega*possQuats.col(i);
	}
	return possNewQuats;
}