#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#include <random>
#include <string>
#include <fstream>
#include <functional>
#include "utils.hpp"
#include "usque.hpp"


namespace Usque {

/*
 * Simulation
 * This class contains parameters for creating a simulation of a rocket's
 * gyro and magnetometer readings. To use this class:
 *
 * 1. Construct a Simulation instance. Set its parameters.
 * 2. Call `run()`, which will populate internal data structures with the
 *    simulation results. 
 * 3. Call `output()`, which will write to the specified file as in CSV format.
 *
 * Usage notes:
 * - Do not call `run()` more than once.
 * - Do not modify simulation parameters during a run, as it is not threadsafe.
 * - Similarly, don't call `output()` while it is running!
 */
class Simulation {
private:
/* Simulation parameters */
	unsigned long rotTime; 
	unsigned long runTime;
	unsigned long gyroDt;
	unsigned long iterations;
	double sigmaBias;
	double sigmaNoise;
	double sigmaMag;
	double a;
	double lambda;
	double f;
	Eigen::Vector3d gyroBias; //Gyro bias
	bool hasFinished;
/* Randomness */
	std::random_device rd;
	std::mt19937 gen;
	std::uniform_real_distribution<double> dis;
/* Simulation variables */
	Usque::Matrix_6x6d covariance; //Covariance matrix
	Usque::Matrix_6x6d noiseCov; //Noise cov
	Eigen::Vector3d rotVec; //Rotational vector
	Eigen::Vector4d attitudeQuat; //Attitude quaternion
	Usque::Vector6d error; //Error
	Eigen::Vector3d idealAngV; //The ideal orientation
	Eigen::Vector4d trueOrient; //The real orientation
	Eigen::Vector3d gyroMeas; //Current gyro reading
	Eigen::Vector3d bias; //defined in readGyro.m; TODO: document better
	Eigen::Vector3d magField; //magnetic field in WMM
	Eigen::Vector3d magMeas; //Current magnetometer value
/* Simulation records */
	std::vector<double> gyroVals;     //Store gyro readings here. Triples.
	std::vector<double> idealGyroVals; //Store "ideal" gyro values here. Triples.
	std::vector<double> magVals;       //Store magnetic field readings. Triples.
	std::vector<double> errorQuats;    //Store error quaternions here. Quats.
/* Testing friends */
#ifdef TESTING
	friend UsqueTests::SimWrapper;
#endif
public:

	/*
	 * Initialize a simulation with default params.
	 * rotTime = 5400 seconds
	 * runTime = 32400 seconds
	 * gyroDt  = 10 seconds
	 * sigmaBias = 3.1623e-10 rad per s^(3/2)
	 * sigmaNoise = 3.1623e-6 rad per s^(1/2)
	 * sigmaMag = 50e-9 Tesla
	 * a = 1 (0 <= a <= 1)
	 * lambda = 1
	 * f = 2 * (a + 1)
	 */
	Simulation();

	//Don't want to allow copies
	Simulation(Simulation& other) = delete;

	//Don't want to allow moves
	Simulation(Simulation&& other) = delete;

	//Set the params.
	void setParams(std::function<void(Simulation&)> builder);

	/*
	 * Runs the simulation. Locks the state.
	 */
	void run();

	/*
	 * Outputs the data stored. Not threadsafe.
	 * Delimiter is `,`, line delimiter is `\n`.
	 * Note that carriage return (used by Windows) is `\r\n`!
	 */
	void output(std::string& filename);

	/*
	 * Outputs the data stored. Not threadsafe.
	 * Delimiter is `,`, line delimiter is `\n`.
	 * Note that carriage return (used by Windows) is `\r\n`!
	 */
	void output(const char* filename);

	/*
	 * Prints the output to stdout. Not threadsafe.
	 * Mainly for use to pipe to another program.
	 */
	void printOutput();

	/*
	 * Outputs the data as a binary file. Each data line consists
	 * of 15 8-byte blocks, followed by a `\n` to indicate the next
	 * entry. No headers are included. The data is formatted like follows:
	 * Iteration (8)
	 * Time (8)
	 * Ideal gyros (8 x 3)
	 * Gyros (8 x 3)
	 * Mags (8 x 3)
	 * Errors (8 x 4)
	 * newline (1)
	 */
	void binOutput();

private:
	
	/*
	 * idealPath.m
	 * Simulates the flight of the spacecraft and gets true angular velocity and
	 * ideal orientation.
	 * TODO: Document params
	 */
	void updateIdealPath(); 

	/*
	 * readWMM.m
	 * Simulates a read to the expected magnetic field measurement.
	 * TODO: We should pass in the position of the spacecraft to get the magnetic
	 * field as a function rather than a constant.
	 */ 
	void readWMM();

	/*
	 * readGyro.m
	 * Simulates gyro measurements.
	 * 
	 * idealAngV: True angular velocity of the spacecraft
	 * bias: The bias to be updated with a zero mean Gaussian white-noise process
	 * gyroMeas: The variable to write the gyro measurement to
	 */
	void readGyro();

	/*
	 * readMag.m
	 * Simulates magnetometer measurements.
	 */	
	void readMag();

	//gyroIntegrate.m
	void gyroIntegrate();

};


}
#endif //SIMULATION_HPP