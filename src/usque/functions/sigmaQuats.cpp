#include "usque.hpp"


/*
	Corresponds to sigmaQuats:1-33
	lambda = constant
	covariance = 6x6 matrix
 */
MatrixErr6 chiValues(
	const int lambda, 
	MatrixNd& covariance, 
	MatrixNd& noiseCov, 
	VectorNd& error
) {
	MatrixNd cholInput = (__N__ + lambda)*(covariance + noiseCov);

	// take the Cholesky decomposition such that cholInput = sigma * 
	//		transpose(sigma)
	// same as in the paper, should not require transposing output as in matlab
	MatrixNd sigma = cholInput.llt().matrixL();
	Eigen::Matrix<double, __N__, 2 * __N__ + 1> chi;
	VectorNd zeros;
	chi << sigma, (-1*sigma), zeros;
	chi = chi + error.replicate<1, 2 * __N__ + 1>();
	return chi;
}

Eigen::Matrix<double, 4, 2 * __N__ + 1> quatDistribution(
	int a, 
	int f, 
	MatrixErr6& chi, 
	Eigen::Vector4d& attitudeQuat
) {
	Eigen::Matrix<double, 4, 2 * __N__> dqK;
	for(int i = 0; i < 2 * __N__; i++) {
		dqK(3,i) = (-a * chi.block(0,i,3,1).squaredNorm() + f * sqrt(f*f + 
		(1-a*a)*chi.block(0,i,3,1).squaredNorm()))/(f*f + chi.block(0,i,3,1).
			squaredNorm());
		dqK.block(0,i,3,1) = ((a + dqK(3,i))/f)*chi.block(0,i,3,1);
	}
	// cerr << "dqK: \n" << dqK << endl;
	Eigen::Matrix<double, 4, 2 * __N__ + 1> possQuats;
	for(int i = 0; i < 2 * __N__; i++) {
		possQuats.col(i) = kalmanArrayMult(dqK.col(i), attitudeQuat);
	}
	possQuats.col(2* __N__) = attitudeQuat;
	// cerr << "possQuats: " << endl << possQuats << endl;
	return possQuats;
}