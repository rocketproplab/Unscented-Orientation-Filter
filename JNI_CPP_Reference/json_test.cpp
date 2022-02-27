#include <cstdlib>
#include <fstream>
#include <iostream>
#include "includes/json.hpp"
using namespace std;


std::vector<std::vector<double>> processNoise(double gyroDt,double sigma_noise,
	double sigma_bias) {
    std::vector<std::vector<double>> noiseCov(6, std::vector<double>(6));
    for(int i = 0; i < 6; i++) {
        if(i < 3) {
			cerr << "yo" << endl;
            noiseCov[i][i] = (gyroDt/2)*(sigma_noise*sigma_noise - sigma_bias*
                    sigma_bias*gyroDt*gyroDt/6);
			cerr << "suh" << endl;
        }
        else {
            noiseCov[i][i] = (gyroDt/2)*sigma_bias*sigma_bias;
        }
    }
	cerr << "noiseCov: " << endl;
	for(int i = 0; i < 6; i++) {
		for(int j = 0; j < 6; j++) {
			cerr << noiseCov[i][j] << " ";
		}
		cerr << endl;
	}
    return noiseCov;
}



int main(int argc, char* argv[]) {
	
    const int a = 1;
    const int lambda = 1;
    const int f = 4;
    const int n = 6;

    string fileName = "imu_params.txt";  
    // Check the number of parameters
    if(argc != 1 and argc != 3) {
        // Tell the user how to run the program
        std::cerr << "Usage: " << argv[0] << " [-f FILENAME]" << std::endl;
        return 1;
    }

    if(argc == 3) {
        // If they are passing a parameter file name, read the file
        if(strcmp(argv[1],"-f") == 0) {
            fileName = argv[2];
        }
        // If the command line flag is unrecognized, throw an error
        else {
            std::cerr << "Usage: " << argv[0] << " [-f FILENAME]" << std::endl;
            return 1;
        }
    }

	// read in the calculation parameter json object
    std::ifstream paramFile(fileName);
    nlohmann::json Params;
	cerr << "hi" << endl;
    paramFile >> Params;
	cerr << "hey" << endl;

    // read in the estimated initial rocket orientation quaternion
    auto attitudeQuat = Params["attitudeQuat"].get<std::vector<double>>();
	cerr << "hello" << endl;

    // read in the estimated initial covariance
    auto covariance = Params["covariance"].get
            <std::vector<std::vector<double>>>();

    // read in the estimated initial gyro bias
    auto gyroBias = Params["gyroBias"].get<std::vector<double>>();

    // read in the gyro sampling interval
    auto gyroDt = Params["gyroDt"].get<double>();
    // read in the gyro bias standard deviation
    auto sigma_bias = Params["sigma_bias"].get<double>();
    // read in the gyro noise standard deviation
    auto sigma_noise = Params["sigma_noise"].get<double>();

    // read in the magnetometer noise standard deviation
    auto sigma_mag = Params["sigma_mag"].get<double>();

	cerr << "whattup" << endl;

    // generate the process noise covariance matrix
    std::vector<std::vector<double>> noiseCov = processNoise(gyroDt,
            sigma_noise,sigma_bias);
	
	return 0;

}



/* std::vector<std::vector<double,6>,13> sigmaChis(lambda,covariance,noiseCov,
        gyroError,a,f,attitudeQuat,n) {} */


