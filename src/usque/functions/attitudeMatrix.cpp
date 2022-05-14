#include "usque.hpp"
namespace RPL {
namespace USQUE{


typedef Eigen::Matrix<double, 3, 4> Matrix_3x4d;
typedef Eigen::Matrix<double, 4, 3> Matrix_4x3d;

/*
	The attitudeMatrix
 */
Eigen::Matrix3d attitudeMatrix(
	Eigen::Vector4d&     quat
) {
	Matrix_3x4d eps;
	Matrix_4x3d psi;
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