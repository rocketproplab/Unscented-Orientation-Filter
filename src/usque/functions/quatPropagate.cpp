#include "usque.hpp"

namespace RPL {
namespace USQUE {

Matrix_4x13d quatPropagate(
	Matrix_4x13d& possQuats, 
	Matrix_3x13d& possAngV, 
	double gyroDt
) {
	Matrix_4x13d possNewQuats;
	for(int i = 0; i < 2 * SIZE + 1; i++) {
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

}
}