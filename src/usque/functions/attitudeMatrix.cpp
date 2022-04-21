#include "Eigen/Cholesky"
namespace RPL {
namespace USQUE{

/*
	The attitudeMatrix
 */
Eigen::Matrix3d attitudeMatrix(Eigen::Vector4d& quat) {
	Eigen::Matrix<double, 3, 4> eps;
	Eigen::Matrix<double, 4, 3> psi;
	//Eq. 16a, expanded.
	eps <<  quat(3),  quat(2), -quat(1), -quat(0),
	       -quat(2),  quat(3),  quat(0), -quat(1),
		    quat(1), -quat(0),  quat(3), -quat(2);

	//Eq. 16b, expanded.
	psi <<  quat(3), -quat(2),  quat(1),
	        quat(2),  quat(3), -quat(0),
		   -quat(1),  quat(0),  quat(3),
		   -quat(0), -quat(1), -quat(2);
	return eps * psi;
}
}
}