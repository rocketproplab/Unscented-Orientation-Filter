#include "Eigen/Cholesky"

#define __N__ 6

extern Eigen::Vector4d kalmanArrayInv(Eigen::Vector4d quat);

extern Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d vecLeft, 
	Eigen::Vector4d vecRight
);

/*
	Corresponds to sigmaQuats:1-33
	lambda = constant
	covariance = 6x6 matrix
 */
typedef Eigen::Matrix<double, 6, 6> Matrix6d
Eigen::MatrixXd chiValues(
	int lambda, 
	Eigen::Matrix6d& covariance, 
	Eigen::Matrix6d& noiseCov, 
	Eigen::Vector4d& error
) {
	Eigen::Matrix6d cholInput = (n+lambda)*(covariance+noiseCov);

	// take the Cholesky decomposition such that cholInput = sigma * 
	//		transpose(sigma)
	// same as in the paper, should not require transposing output as in matlab
	MatrixXd sigma = cholInput.llt().matrixL();
	Matrix<double, __N__, 2 * __N__ + 1> chi;
	MatrixXd zeros = MatrixXd::Zero(n,1);
	chi << sigma, (-1*sigma), zeros;
	chi = chi + error.replicate<1,2*n+1>();
	return chi;
}

Eigen::MatrixXd quatDistribution(
	int a, 
	int f, 
	Eigen::MatrixXd& chi, 
	Eigen::Vector4d& attitudeQuat) {
	using namespace Eigen;
	const int n = 6;
	MatrixXd dqK(4,2*n);
	for(int i = 0; i < 2*n; i++) {
		dqK(3,i) = (-a * chi.block(0,i,3,1).squaredNorm() + f * sqrt(f*f + 
		(1-a*a)*chi.block(0,i,3,1).squaredNorm()))/(f*f + chi.block(0,i,3,1).
			squaredNorm());
		dqK.block(0,i,3,1) = ((a + dqK(3,i))/f)*chi.block(0,i,3,1);
	}
	// cerr << "dqK: \n" << dqK << endl;
	MatrixXd possQuats(4,2*n+1);
	for(int i = 0; i < 2*n; i++) {
		possQuats.col(i) = kalmanArrayMult(dqK.col(i), attitudeQuat);
	}
	possQuats.col(2*n) = attitudeQuat;
	// cerr << "possQuats: " << endl << possQuats << endl;
	return possQuats;
}