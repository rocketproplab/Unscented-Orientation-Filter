#include "simulation.hpp"
#include <cstdio>
#define PI 3.14159265359

namespace Usque {

//Default params	
Simulation::Simulation() :
	rotTime(5400),
	runTime(32400),
	gyroDt(10),
	sigmaBias(3.1623e-10),
	sigmaNoise(0.31623e-6),
	sigmaMag(50e-9),
	a(1),
	lambda(1),
	f(4),
	iterations(runTime / gyroDt),
	hasFinished(false)
{
	/* Construct gyro bias */
	const double GB = (PI / 6.48) * 1e-5;
	gyroBias << GB, GB, GB;
	/* Initialize states */
	rotVec << 1, 2, 3;
	trueOrient << 0, 0, 0, 1;
	attitudeQuat << 0, 0, 0, 1;
	/* Construct initial error */
	error << 0, 0, 0, GB, GB, GB;
	/* Construct covariance */
	const double attErr = (PI / 360) * (PI / 360);
	const double biasErr = (PI / 3.24e6) * (PI / 3.24e6);
	covariance.diagonal() << attErr, attErr, attErr, biasErr, biasErr, biasErr;
	/* Construct noise covariance */
	const int A = sigmaNoise * sigmaNoise - (1 / 6) * sigmaBias * sigmaBias * gyroDt * gyroDt;
	noiseCov.topLeftCorner<3, 3>() << Eigen::Matrix3d::Identity() * A * gyroDt / 2;
	noiseCov.topRightCorner<3, 3>() << Eigen::Matrix3d::Zero();
	noiseCov.bottomLeftCorner<3, 3>() << Eigen::Matrix3d::Zero();
	noiseCov.bottomRightCorner<3, 3>() << Eigen::Matrix3d::Identity() * (sigmaBias * sigmaBias);
	/* Reserve space for simulation data */
	gyroVals.reserve(3 * iterations);
	idealGyroVals.reserve(3 * iterations);
	magVals.reserve(3 * iterations);
	errorQuats.reserve(4 * iterations);
	/* Randomness initialization */
	rng.seed(0);//Set the seed of the random number generator.

}

void Simulation::run() {
	// TODO: Could add synchronizing logic here
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
		errorQuats.push_back(error(0));
		errorQuats.push_back(error(1));
		errorQuats.push_back(error(2));
		errorQuats.push_back(error(3));
		//Reset error (runFilter.m:266)
		error.head<3>() << 0, 0, 0;
	}
}

void Simulation::output(std::string& filename) {
	output(filename.c_str());
}


void Simulation::output(const char* filename) {
	//Output as a csv file
	auto file = std::fopen(filename, "w");
	std::fprintf(file, "Iteration,Time,idealX,idealY,idealZ,gyroX,gyroY,gyroZ,magX,magY,magZ,errorX,errorY,errorZ,errorW\n");
	for(unsigned long i = 0; i < iterations; ++i) {
		std::fprintf(file, "%lu,%lu,", i, (i * gyroDt)); //Time, Iteration
		std::fprintf(file, "%.6lf,%.6lf,%.6lf,",        idealGyroVals[i * 3 + 0],  //idealX
		                                                idealGyroVals[i * 3 + 1],  //idealY
		                                                idealGyroVals[i * 3 + 2]); //idealZ
		std::fprintf(file, "%.6lf,%.6lf,%.6lf,",        gyroVals[i * 3 + 0],       //gyroX
		                                                gyroVals[i * 3 + 1],       //gyroY
		                                                gyroVals[i * 3 + 2]);      //gyroZ
		std::fprintf(file, "%.6lf,%.6lf,%.6lf,",        magVals[i * 3 + 0],        //magX
		                                                magVals[i * 3 + 1],        //magY
		                                                magVals[i * 3 + 2]);       //magZ
		std::fprintf(file, "%.6lf,%.6lf,%.6lf,%.6lf\n", errorQuats[i * 4 + 0],     //errorQuatsX
		                                                errorQuats[i * 4 + 1],     //errorQuatsY
		                                                errorQuats[i * 4 + 2],     //errorQuatsZ
		                                                errorQuats[i * 4 + 3]);    //errorQuatsW
	}
	//Flush so results are printed
	std::fflush(file);
	std::fclose(file);
}


void Simulation::printOutput() {
	//Header
	std::printf("Iteration,Time,idealX,idealY,idealZ,gyroX,gyroY,gyroZ,magX,magY,magZ,errorX,errorY,errorZ,errorW\n");
	for(unsigned long i = 0; i < iterations; ++i) {
		std::printf("%lu,%lu,", i, (i * gyroDt)); //Time, Iteration
		std::printf("%.6lf,%.6lf,%.6lf,",        idealGyroVals[i * 3 + 0],  //idealX
		                                         idealGyroVals[i * 3 + 1],  //idealY
		                                         idealGyroVals[i * 3 + 2]); //idealZ
		std::printf("%.6lf,%.6lf,%.6lf,",        gyroVals[i * 3 + 0],       //gyroX
		                                         gyroVals[i * 3 + 1],       //gyroY
		                                         gyroVals[i * 3 + 2]);      //gyroZ
		std::printf("%.6lf,%.6lf,%.6lf,",        magVals[i * 3 + 0],        //magX
		                                         magVals[i * 3 + 1],        //magY
		                                         magVals[i * 3 + 2]);       //magZ
		std::printf("%.6lf,%.6lf,%.6lf,%.6lf\n", errorQuats[i * 4 + 0],     //errorQuatsX
		                                         errorQuats[i * 4 + 1],     //errorQuatsY
		                                         errorQuats[i * 4 + 2],     //errorQuatsZ
		                                         errorQuats[i * 4 + 3]);    //errorQuatsW
	}
	//Flush so results are printed
	std::fflush(stdout);

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
	gyroBias += sigmaBias * Eigen::Vector3d::NullaryExpr(3, [&](){return 5 * dis(rng);});
	Eigen::Vector3d noise = sigmaNoise * Eigen::Vector3d::NullaryExpr(3, [&](){return 5 * dis(rng);});
	gyroMeas = idealAngV + gyroBias + noise;
}

void Simulation::readMag() {
	//Generate noise
	Eigen::Vector3d noise = sigmaMag * Eigen::Vector3d::NullaryExpr(3, [&](){return 5 * dis(rng);}); 
	Eigen::Vector4d worldQuat;
	worldQuat << magField, 0;
	Eigen::Vector4d idealMagMeas = multQuat(multQuat(invQuat(trueOrient), worldQuat), trueOrient);
	magMeas = idealMagMeas.head<3>() + noise;
}

void Simulation::gyroIntegrate() {
	const double norm = idealAngV.norm();
	Eigen::Vector4d deltaQuat = rotQuat(gyroDt * norm, idealAngV / (norm));
	trueOrient = multQuat(trueOrient, deltaQuat);
}

}