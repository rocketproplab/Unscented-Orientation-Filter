#ifndef SIM_H
	#define SIM_H
	#include "Eigen/Core"

namespace Usque  {
//multQuat.m
Eigen::Vector4d multQuat(
	Eigen::Vector4d& q, 
	Eigen::Vector4d& p
);

Eigen::Vector4d multQuat(
	Eigen::Vector4d&& q, 
	Eigen::Vector4d& p
);

Eigen::Vector4d multQuat(
	Eigen::Vector4d& q, 
	Eigen::Vector4d&& p
);

Eigen::Vector4d multQuat(
	Eigen::Vector4d&& q, 
	Eigen::Vector4d&& p
);

Eigen::Vector4d invQuat(
	Eigen::Vector4d& quat
);

Eigen::Vector4d invQuat(
	Eigen::Vector4d&& quat
);

Eigen::Vector4d conjQuat(
	Eigen::Vector4d& quat
);

Eigen::Vector4d rotQuat(
	double theta,
	Eigen::Vector3d& quat
);

Eigen::Vector4d rotQuat(
	double theta,
	Eigen::Vector3d&& quat
);

}
#endif