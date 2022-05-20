#ifndef SIMULATION_HPP
#define SIMULATION_HPP
#include <optional>
#include <random>
#include "utils.hpp"
#include "usque.hpp"
namespace Usque {
/*
 * Simulation
 * This class contains parameters for creating a simulation of a rocket's
 * gyro and magnetometer readings. To use this class:
 *
 * 1. Construct a Simulation instance. To do this, use a SimBuilder.
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
	int rotTime = 5400;
	int runTime = 32400;
	int gyroDt = 10;
	double sigmaBias = 3.1623e-10;
	double sigmaNoise = 0.31623e-6;
	double sigmaMag = 50e-9;
	int a = 1;
	int lambda = 1;
	int f = 2 * (a + 1);
	int iterations = runTime / gyroDt;
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
public:

	/*
	 * Initialize a simulation with default params.
	 * TODO: What are default params?
	 */
	Simulation();

	void setParam();

	/*
	 * Runs the simulation. Not threadsafe.
	 */
	void run();

	/*
	 * Outputs the data stored. Not threadsafe.
	 */
	void output();

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

/*
 * Use this class to build a simulation with appropriate params. 
 */
class SimBuilder {
public:
	std::optional<int> rotTime;
	std::optional<int> runTime;
	std::optional<int> gyroDt;
	std::optional<double> sigmaBias; 
	std::optional<double> sigmaNoise;
	std::optional<double> sigmaMag;
	std::optional<int> a;
	std::optional<int> lambda;
	std::optional<int> f;
	std::optional<int> iterations; 
	std::optional<Eigen::Vector3d> gyroBias; //Gyro bias

	Simulation build();
};

}
#endif