#ifndef MEAS_H
	#define MEAS_H
	#include "Eigen/Core"

namespace RPL {
namespace Measure {

//idealPath.m
void idealPath(
	const int            rotTime, 
	Eigen::Vector3d&     rotVec, 
	const int            gyroDt,
	Eigen::Vector4d&     orientationQuat,
	Eigen::Vector3d&     idealAngV
);

//readWMM.m
Eigen::Vector3d readWMM();

//readGyro.m
void readGyro(
	Eigen::Vector3d& idealAngV,
	Eigen::Vector3d& bias,
	Eigen::Vector3d& gyroMeas
);

}
}

#endif