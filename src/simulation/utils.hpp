#ifndef SIM_H
	#define SIM_H
	#include "Eigen/Core"

namespace RPL {
namespace Sim {

//multQuat.m
Eigen::Vector4d multQuat(
	Eigen::Vector4d& q, 
	Eigen::Vector4d& p
);

//gyroIntegrate.m
void gyroIntegrate(
	Eigen::Vector4d&     initialQ,
	Eigen::Vector3d&     gyroMeas,
	const int            gyroDt
);


}
}


#endif