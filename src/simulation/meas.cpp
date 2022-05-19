#include "meas.hpp"
#include <random>

namespace RPL {
namespace Meas {

std::random_device rd;
std::mt19937 gen(rd);
std::uniform_real_distribution<double> dis(-1.0, 1.0);


void idealPath(
	const int            rotTime, 
	Eigen::Vector3d&     rotVec, 
	const int            gyroDt,
	Eigen::Vector4d&     orientationQuat,
	Eigen::Vector3d&     idealAngV
) {
	idealAngV = (2 * PI / (rotTime * rotVec.norm())) * rotVec;
	gyroIntegrate(orientationQuat, idealAngV, gyroDt);
}

//TODO: Get the magnetic field at a position. Use a real map.
void readWMM(Eigen::Vector3d magField) {
	magField << 22819e-9, 4638.9e-9, -39574.9e-9;
}

void readGyro(
	Eigen::Vector3d& idealAngV,
	Eigen::Vector3d& bias,
	const double     sigmaBias,
	const double     sigmaNoise,
	Eigen::Vector3d& gyroMeas,
) {

	bias += sigmaBias * Eigen::Vector3d::NullaryExpr(3, 3, [&](){return dis(gen);});
	Eigen::Vector3d noise = sigmaNoise * Eigen::Vector3d::NullaryExpr(3, 3, [&](){return dis(gen);});
	gyroMeas << idealAngV + bias + noise;
}

}
}