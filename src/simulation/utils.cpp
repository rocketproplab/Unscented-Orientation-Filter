#include "sim_utils.hpp"

namespace RPL {
namespace Sim {

Eigen::Vector4d multQuat(
	Eigen::Vector4d& q, 
	Eigen::Vector4d& p
) {
	double a = q(4) * p(4) - q(1) * p(1) - q(2) * p(2) - q(3) * p(3);
	double b = q(4) * p(2) - q(1) * p(3) + q(2) * p(4) + q(3) * p(1);
	double c = q(4) * p(3) + q(1) * p(2) - q(2) * p(1) + q(3) * p(4);
	double d = q(4) * p(1) + q(1) * p(4) + q(2) * p(3) - q(3) * p(2);
 
	Eigen::Vector4d result;
	result << a, b, c, d;
	return result;
}

void gyroIntegrate(
	Eigen::Vector4d&     initialQ,
	Eigen::Vector3d&     gyroMeas,
	const int            gyroDt
) {
	const double norm = gyroMeas.norm();
	Eigen::Vector4d deltaQuat = rotQuat(gyroDt * norm, gyroMeas / norm);
	initialQ = multQuat(initialQ, deltaQuat);
}


}
}