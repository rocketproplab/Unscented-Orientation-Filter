#include "runFilter.h"
#include "lib/Eigen/Cholesky"
#include "lib/Eigen/LU"

void runFilter(
	Eigen::Vector4d& attitudeQuat, 
	Eigen::MatrixXd& covariance, 
	Eigen::Vector3d& gyroBias,
	Eigen::Matrix4f& errorResult
) {
	int rotation = 5400;
	Eigen::Vector3d rotationVec(1, 2, 3);
	int runTime = 32400;
	int gyroDt = 10;
	double sigmaBias = 3.1623E-10;
	double sigmaNoise = 3.1623E-5;
	double sigmaMag = 50E-9;

	const int A = 1;
	const int LAMBDA = 1;
	const int F = 2 * (A + 1);
	const int N = 6;

	int iterations = (runTime / gyroDt);
	;
	
}