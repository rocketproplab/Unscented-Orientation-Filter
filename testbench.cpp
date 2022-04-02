#include "USQUE.h"
#include "filter_step.h"
#include <cstdlib>
#include <iostream>
#include "lib/json.hpp"
#include "lib/Eigen/Cholesky"
#include "lib/Eigen/LU"

using namespace std;
int main(int argc, char* argv[]) {
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


