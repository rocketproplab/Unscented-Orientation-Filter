#ifndef MEAS_H
	#define MEAS_H
	#include "Eigen/Core"

/*
 * meas.hpp
 * This file contains method declarations for all of the measurement
 * functionality of the simulation.
 */
namespace RPL {



/*
 * idealPath.m
 * Simulates the flight of the spacecraft and gets true angular velocity and
 * ideal orientation.
 * TODO: Document params
 */
void idealPath(
	const int            rotTime, 
	Eigen::Vector3d&     rotVec, 
	const int            gyroDt,
	Eigen::Vector4d&     orientationQuat,
	Eigen::Vector3d&     idealAngV
);

/*
 * readWMM.m
 * Simulates a read to the expected magnetic field measurement.
 * TODO: We should pass in the position of the spacecraft to get the magnetic
 * field as a function rather than a constant.
 */ 
Eigen::Vector3d readWMM();

/*
 * readGyro.m
 * Simulates gyro measurements. Adds noise to the measurement "pseudo-randomly".
 * 
 * idealAngV: True angular velocity of the spacecraft
 * bias: The bias to be updated with a zero mean Gaussian white-noise process
 * gyroMeas: The variable to write the gyro measurement to
 */
void readGyro(
	Eigen::Vector3d& idealAngV,
	Eigen::Vector3d& bias,
	Eigen::Vector3d& gyroMeas
);

}

#endif