#include "usque.hpp"

//predictError.m: 18
VectorNd predictError(
	const int lambda, 
	Eigen::Matrix<double, __N__, 2 * __N__ + 1>& possNewError, 
	MatrixNd& noiseCov
) {
	VectorNd predError;
	predError << (lambda * possNewError.col(2*__N__) + 0.5*possNewError.
		leftCols(2 * __N__).rowwise().sum())/(__N__ + lambda);
	// cerr << "predError: " << endl << predError << endl;
	return predError;
}

//predictError.m: 19-27
MatrixNd predictCov(
	const int lambda, 
	Eigen::Matrix<double, __N__, 2 * __N__ + 1>& possNewError, 
	MatrixNd& noiseCov, 
	VectorNd& predError
) {
	Eigen::Matrix<double, __N__, 2 * __N__> intermediate;
	intermediate = possNewError.leftCols(2 * __N__) - predError.replicate<1,2 * __N__>();
	MatrixNd predCov;
	predCov << (lambda*(possNewError.col(2*__N__) - predError)*(possNewError.
		col(2*__N__) - predError).transpose() + 0.5*intermediate*intermediate.
		transpose())/(__N__ + lambda) + noiseCov;
	return predCov;
}
