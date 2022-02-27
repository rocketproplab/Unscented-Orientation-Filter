//
// Created by Matthew Mikhailov on 1/23/21.
//
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include "lib/json.hpp"
using namespace std;

std::vector<std::vector<double>> processNoise(double gyroDt,double sigma_noise,
	double sigma_bias) {
    std::vector<std::vector<double>> noiseCov(6, std::vector<double>(6));
    for(int i = 0; i < 6; i++) {
        if(i < 3) {
            noiseCov[i][i] = (gyroDt/2)*(sigma_noise*sigma_noise - sigma_bias*
                    sigma_bias*gyroDt*gyroDt/6);
        }
        else {
            noiseCov[i][i] = (gyroDt/2)*sigma_bias*sigma_bias;
        }
    }
    return noiseCov;
}

int main(int argc, char** argv) {

	string fileName = "imu_params.txt";  
    // Check the number of parameters
    if(argc != 1 and argc != 3) {
        // Tell the user how to run the program
        std::cerr << "Usage:" << argv[0] << "[-f FILENAME]" << std::endl;
        return 1;
    }

    if(argc == 3) {
        // If they are passing a parameter file name, create the file
        if(strcmp(argv[1],"-f") == 0) {
            fileName = argv[2];
        }
        // If the command line flag is unrecognized, throw an error
        else {
            std::cerr << "Usage:" << argv[0] << "[-f FILENAME]" << std::endl;
            return 1;
        }
    }
    // Create an empty structure
    nlohmann::json j;

	//--------------------------------------------------------------------------
	// Edit parameters here
	//--------------------------------------------------------------------------

    vector<double> attitudeQuat = {0,0,0,1};
    double attitude_error = pow(M_PI/360,2);
    double bias_error = pow((M_PI/3.24)*1e-6,2);
    vector<vector<double>> covariance = {{attitude_error,0,0,0,0,0},{0,attitude_error,0,0,0,0},
                       {0,0,attitude_error,0,0,0},{0,0,0,bias_error,0,0},
                       {0,0,0,0,bias_error,0},{0,0,0,0,0,bias_error}};
	double gyroDt = 10;
	vector<double> gyroBias = {(M_PI/6.48)*1e-5,(M_PI/6.48)*1e-5,(M_PI/6.48)*1e-5};
	double sigma_bias = 3.1623e-10;
	double sigma_noise = 3.1623e-7;
	double sigma_mag = 5e-8;
	vector<vector<double>> noiseCov = processNoise(gyroDt,sigma_noise,sigma_bias);
	vector<double> gyroMeas = {0,0,0};
	vector<double> magMeas = {0,0,0};
	vector<double> magField = {0,0,0};
	vector<double> error = {0,0,0,gyroBias[0],gyroBias[1],gyroBias[2]};

	// Convert to json
	j["attitudeQuat"] = attitudeQuat;
	j["covariance"] = covariance;
	j["gyroDt"] = gyroDt;
    j["gyroBias"] = gyroBias;
	j["sigma_bias"] = sigma_bias;
    j["sigma_noise"] = sigma_noise;
    j["sigma_mag"] = sigma_mag;
	j["noiseCov"] = noiseCov;
	j["gyroMeas"] = gyroMeas;
	j["magMeas"] = magMeas;
	j["magField"] = magField;
	j["error"] = error;

    // Create and write to text file
    ofstream Params(fileName);
    Params << j.dump(4) << std::endl;

    return 0;
}
