#include "usque.hpp"
namespace RPL {
namespace USQUE {


//predictError.m: 18
Vector6d predictError(
	const int lambda, 
	Eigen::Matrix<double, SIZE, 2 * SIZE + 1>& possNewError, 
	Matrix_6x6d& noiseCov
) {
	Vector6d predError;
	predError << (lambda * possNewError.col(2*SIZE) + 0.5*possNewError.
		leftCols(2 * SIZE).rowwise().sum())/(SIZE + lambda);
	// cerr << "predError: " << endl << predError << endl;
	return predError;
}

//predictError.m: 19-27
Matrix_6x6d predictCov(
	const int lambda, 
	Eigen::Matrix<double, SIZE, 2 * SIZE + 1>& possNewError, 
	Matrix_6x6d& noiseCov, 
	Vector6d& predError
) {
	Eigen::Matrix<double, SIZE, 2 * SIZE> intermediate;
	intermediate = possNewError.leftCols(2 * SIZE) - predError.replicate<1,2 * SIZE>();
	Matrix_6x6d predCov;
	predCov << (lambda*(possNewError.col(2*SIZE) - predError)*(possNewError.
		col(2*SIZE) - predError).transpose() + 0.5*intermediate*intermediate.
		transpose())/(SIZE + lambda) + noiseCov;
	return predCov;
}

}
}