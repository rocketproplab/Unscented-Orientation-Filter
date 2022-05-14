// #include <unistd.h>
#include "usque.hpp"
#include "sim_utils.hpp"
#include "meas.hpp"
#include <iostream>
#define PI 3.1415926535d
#define ATT_ERR PI * PI / (360 * 360)
#define BIAS_ERR PI * PI / (3.24E6 * 3.24E6)


/*
	Testbench: Simulation

	Ported from MatLab runFilter.m
 */



//
int main(int argc, char *argv[])
{
	//Initialization
	Eigen::Vector4d attitudeQuat << 0, 0, 0, 1;
	double attErr = (PI / 360) * (PI / 360);
	double biasErr = (PI / 3.24e6) * (PI / 3.24e6);
	Matrix_6x6d covariance = Matrix_6x6d::Zero();
	covariance.diagonal() << attErr, attErr, attErr, biasErr, biasErr, biasErr;
	
#define GBIAS (PI / 6.48) * (1e-5)
	Vector3d gyroBias;
	gyroBias << GBIAS, GBIAS, GBIAS;
	Vector6d error;
	error << 0, 0, 0, GB, GB, GB;
#undef GBIAS
	

	//Simulation params
	int rotTime = 5400;
	Eigen::Vector3d rotVec;
	rotVec << 1, 2, 3;
	int runTime = 32400;
	int gyroDt = 10;
	double sigmaBias = 3.1623e-10;
	double sigmaNoise = 0.31623e-6;
	double sigmaMag = 50e-9;
	//Constants and fine tuning
	const int a = 1;
	const int lambda = 1;
	const int f = 2 * (a + 1);
	const int iterations = runTime / gyroDt;

	//Run Filter
	for(int i = 0; i < iterations; ++i) {
		filterStep(gyroDt, 
		           sigmaBias, 
		           sigmaNoise,
		           sigmaMag, 
		           noiseCov,
		           attitudeQuat,
		           covariance,
		           gyroBias,
		           gyroMeas,
		           magMeas,
		           magField,
		           error
		);
		//Output error

	}
	return 0;
}
