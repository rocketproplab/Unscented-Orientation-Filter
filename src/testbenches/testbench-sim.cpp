#include "filterStep.h"
#include "runFilter.h"
#include "lib/Eigen/Cholesky"
#include "lib/Eigen/LU"

#define PI 3.1415926535d
#define ATT_ERR PI * PI / (360 * 360)
#define BIAS_ERR PI * PI / (3.24E6 * 3.24E6)

//Start the simulation
void startSim() {
	Eigen::Vector4d attitudeQuat(0, 0, 0, 1);
	Eigen::VectorXd covarianceVec {{ATT_ERR, ATT_ERR, ATT_ERR, BIAS_ERR, BIAS_ERR, BIAS_ERR}};
	Eigen::Vector3d gyroBias(PI / 6.48E-5, PI / 6.48E-5, PI / 6.48E-5);
	//att quat 
	//covaraince 
	//error
	Eigen::Matrix4d errorResult;
	runFilter(attitudeQuat,covariance,gyroBias, errorResult);
	
}


//
int main(int argc, char * argv[])
{
	startSim();
    return 0;
}
