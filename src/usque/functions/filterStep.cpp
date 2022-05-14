#include "usque.hpp"
#include "Eigen/LU"
// #include <cstdlib>
// #include <iostream>
namespace RPL {
namespace USQUE {

// JNIEXPORT void JNICALL
// Java_USQUE_filterStep(JNIEnv *env, jclass obj, jstring json_params) {
void filterStep(
	const double gyroDt, 
	const double sigma_bias, 
	const double sigma_noise, 
	const double sigma_mag, 
	Matrix_6x6d& noiseCov, //6x6 
	Eigen::Vector4d& attitudeQuat, 
	Matrix_6x6d& covariance, 
	Eigen::Vector3d& gyroBias,
	Eigen::Vector3d& gyroMeas,
	Eigen::Vector3d& magMeas, 
	Eigen::Vector3d& magField,
	Vector6d& error
) {

/*	Fine tuning parameters described in the research paper. Recommmended values
	are a=1, lambda=1, and f=4 */ 
    const int a = 1;
    const int lambda = 1;
    const int f = 4;
	
/*	Half the number of sampling points for the extended Kalman filter. Should be
	n=6 unless you are modifying the algorithm */

	/* runFilter.m: 167 */
	//sigmaQuats.m: 1-33
	Matrix_6x13d chi = chiValues(lambda, covariance, noiseCov, error);
	//sigmaQuats.m: 34-70
	Matrix_4x13d possQuats = quatDistribution(a,f,chi,attitudeQuat);

	//runFilter.m: 175 (Eq. 35)
	Matrix_3x13d possAngV = sigmaOmegas(gyroMeas, chi);
	//runFilter.m: 183 (Eq. 34)
	Matrix_4x13d possNewQuats = quatPropagate(possQuats,possAngV,gyroDt);
	//runFilter.m: 189
	Matrix_6x13d possNewError = newChis(possNewQuats, chi, f, a);
	//predictError.m: 18
	Vector6d predError = predictError(lambda, possNewError, noiseCov);
	//predictError.m: 19-27
	Matrix_6x6d predCov = predictCov(lambda, possNewError, noiseCov, 
		predError);
	//runFilter.m: 213
	Matrix_3x13d possExpMagMeas = sigmaMeas(possNewQuats, magField);
	
	//runFilter.m: 217
	Eigen::Vector3d predMagMeas = predictMeas(lambda, possExpMagMeas);

	//runFilter.m: 224
	Eigen::Matrix3d newInnovationCov = innovationCov(possExpMagMeas, 
		predMagMeas, lambda, sigma_mag).inverse();

	//runFilter.m: 229
	Matrix_6x3d newCrossCorrelation = crossCorr(possNewError, predError,
		possExpMagMeas, predMagMeas, lambda);

	//updateError.m: 27
	Eigen::Vector3d innovation = magMeas - predMagMeas;
	//updateError.m: 31
	Matrix_6x3d gain = newCrossCorrelation * 
		newInnovationCov;
	//runFilter.m: 257-258
	error = predError + gain*innovation; // Generate new error

	//updateError.m: 41
	covariance = predCov - gain * newInnovationCov * gain.transpose(); // New cov
	//runFilter.m: 251
	attitudeQuat = quatUpdate(error, f, a, possNewQuats); // new attitudeQuat
	//runFilter.m: 266
	error.head(3) << 0,0,0;
	

}

}
}