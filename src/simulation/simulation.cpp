#include "simulation.hpp"
#define PI 3.14159265359
namespace Usque {
	
	
	
Simulation::Simulation() {
	
	/* Construct noise covariance */
	const int A = sigmaNoise * sigmaNoise - (1 / 6) * sigmaBias * sigmaBias * gyroDt * gyroDt;
	noiseCov.topLeftCorner<3, 3>() << Eigen::Matrix3d::Identity() * A * gyroDt / 2;
	noiseCov.topRightCorner<3, 3>() << Eigen::Matrix3d::Zero();
	noiseCov.bottomLeftCorner<3, 3>() << Eigen::Matrix3d::Zero();
	noiseCov.bottomRightCorner<3, 3>() << Eigen::Matrix3d::Identity() * (sigmaBias * sigmaBias);
	


}

void Simulation::run() {
	if(hasFinished) {
		return;
	}

	for(int i = 0; i < iterations; ++i) {
		//Generate the actual ideal angular velocity of the rocket, and
		//precise current orientation of the rocket.
		updateIdealPath();
		//Record the ideal gyro value.
		idealGyroVals.push_back(idealAngV(0));
		idealGyroVals.push_back(idealAngV(1));
		idealGyroVals.push_back(idealAngV(2));

		//Read the gyro and record it.
		readGyro();
		gyroVals.push_back(gyroMeas(1));
		gyroVals.push_back(gyroMeas(1));
		gyroVals.push_back(gyroMeas(2));

		//Read World Magnetic Model.
		readWMM();

		//Read magnetometer, and record it
		readMag();
		magVals.push_back(magMeas(0));
		magVals.push_back(magMeas(1));
		magVals.push_back(magMeas(2));

		filterStep(a, 
		           lambda, 
		           f,
		           gyroDt,
		           sigmaBias, 
		           sigmaNoise,
		           sigmaMag, 
		           noiseCov,
		           attitudeQuat,
		           covariance,
		           gyroMeas,
		           magMeas,
		           magField,
		           error);
		//Output error
		
	}
}
/*
 * Simulate instrument readings
 */
void Simulation::updateIdealPath() {
	this->idealAngV = (2 * PI / (rotTime * rotVec.norm())) * rotVec;
	gyroIntegrate();
}

void Simulation::readWMM() {
	magField << 22819e-9, 4638.9e-9, -39574.9e-9;
}

void Simulation::readGyro() {
	//Generate a random bias.
	this->bias += sigmaBias * Eigen::Vector3d::NullaryExpr(3, 3, [&](){return dis(gen);});
	Eigen::Vector3d noise = sigmaNoise * Eigen::Vector3d::NullaryExpr(3, 3, [&](){return dis(gen);});
	this->gyroMeas << idealAngV + bias + noise;
}

void Simulation::readMag() {
	//Generate noise
	Eigen::Vector3d noise = sigmaMag * Eigen::Vector3d::NullaryExpr(3, 3, [&](){return dis(gen);}); 
	Eigen::Vector4d worldQuat;
	worldQuat << magField, 0;
	Eigen::Vector4d idealMagMeas = multQuat(multQuat(invQuat(trueOrient), worldQuat), trueOrient);
	magMeas = idealMagMeas.head<3>() + noise;
}

void Simulation::gyroIntegrate() {
	const double norm = gyroMeas.norm();
	Eigen::Vector4d deltaQuat = rotQuat(gyroDt * norm, gyroMeas / (norm));
	trueOrient = multQuat(trueOrient, deltaQuat);
}

}