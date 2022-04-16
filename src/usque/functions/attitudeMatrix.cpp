#include "Eigen/Cholesky"

Eigen::Matrix3d attitudeMatrix(Eigen::Vector4d& quat) {
	Eigen::Matrix<double, 3, 4> eps;
	Eigen::Matrix<double, 4, 3> psi;
	/*
	 * Block matrix:
	 * [
	 *   I * crossMatrix(quat[0:2]) 
	 *   -1 * quat[0:2].transpose()
	 * ]
	 */
	eps <<  quat(3),  quat(2), -quat(1), -quat(0),
	       -quat(2),  quat(3),  quat(0), -quat(1),
		    quat(1), -quat(0),  quat(3), -quat(2);

	psi <<  quat(3), -quat(2),  quat(1),
	        quat(2),  quat(3), -quat(0),
		   -quat(1),  quat(0),  quat(3),
		   -quat(0), -quat(1), -quat(2);
	return eps * psi;
}