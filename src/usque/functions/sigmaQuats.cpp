#include "usque.hpp"
#include "Eigen/Cholesky"
namespace RPL {
namespace USQUE {

/*
	Corresponds to sigmaQuats:1-33
	lambda = constant
	covariance = 6x6 matrix
 */
Matrix_6x13d chiValues(
	const int            lambda, 
	Matrix_6x6d&         covariance, 
	Matrix_6x6d&         noiseCov, 
	Vector6d&            error
) {
	Matrix_6x6d cholInput = (SIZE + lambda)*(covariance + noiseCov);

	// take the Cholesky decomposition such that cholInput = sigma * 
	//		transpose(sigma)
	// same as in the paper, should not require transposing output as in matlab
	Matrix_6x6d sigma = cholInput.llt().matrixL();
	Matrix_6x13d chi;
	chi << sigma, (-1*sigma), Vector6d::Zero();
	chi = chi + error.replicate<1, 2 * SIZE + 1>();
	return chi;
}

Matrix_4x13d quatDistribution(
	int a, 
	int f, 
	Matrix_6x13d& chi, 
	Eigen::Vector4d& attitudeQuat
) {
	Matrix_4x13d dqK;
	for(int i = 0; i < 2 * SIZE; i++) {
		dqK(3,i) = (-a * chi.block(0,i,3,1).squaredNorm() + f * sqrt(f*f + 
		(1-a*a)*chi.block(0,i,3,1).squaredNorm()))/(f*f + chi.block(0,i,3,1).
			squaredNorm());
		dqK.block(0,i,3,1) = ((a + dqK(3,i))/f)*chi.block(0,i,3,1);
	}
	// cerr << "dqK: \n" << dqK << endl;
	Eigen::Matrix<double, 4, 2 * SIZE + 1> possQuats;
	for(int i = 0; i < 2 * SIZE; i++) {
		possQuats.col(i) = kalmanArrayMult(dqK.col(i), attitudeQuat);
	}
	possQuats.col(2* SIZE) = attitudeQuat;
	// cerr << "possQuats: " << endl << possQuats << endl;
	return possQuats;
}

}
}