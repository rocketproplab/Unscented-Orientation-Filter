#include <cmath>
#include "utils.hpp"

namespace Usque {

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

Eigen::Vector4d multQuat(
	Eigen::Vector4d&& q, 
	Eigen::Vector4d& p
) {
	return multQuat(p, q);
}

Eigen::Vector4d multQuat(
	Eigen::Vector4d& q, 
	Eigen::Vector4d&& p
) {
	return multQuat(p, q);
}

Eigen::Vector4d multQuat(
	Eigen::Vector4d&& q, 
	Eigen::Vector4d&& p
) {
	return multQuat(p, q);
}

Eigen::Vector4d invQuat(
	Eigen::Vector4d& quat
) {
	double norm = quat.norm();
	Eigen::Vector4d newQuat = conjQuat(quat)/ (norm * norm);
	return newQuat;
}

Eigen::Vector4d invQuat(
	Eigen::Vector4d&& quat
) {
	return invQuat(quat);
}

Eigen::Vector4d conjQuat(
	Eigen::Vector4d& quat
) {
	Eigen::Vector4d conjugate;
	conjugate << -quat.head<3>(), quat(3);
	return conjugate;
}

Eigen::Vector4d rotQuat(
	double theta,
	Eigen::Vector3d& quat
) {
	Eigen::Vector3d normalized = quat.normalized();
	Eigen::Vector4d result;
	result << normalized.head<3>() * std::sin(theta / 2), std::cos(theta / 2);
	return result;
}

Eigen::Vector4d rotQuat(
	double theta,
	Eigen::Vector3d&& quat
) {
	return rotQuat(theta, quat);
}
}
