#include "usque.hpp"
// #include <cstdlib>
// #include <iostream>

// JNIEXPORT void JNICALL
// Java_USQUE_filterStep(JNIEnv *env, jclass obj, jstring json_params) {
void filterStep(
	const double gyroDt, 
	const double sigma_bias, 
	const double sigma_noise, 
	const double sigma_mag, 
	MatrixNd& noiseCov, //6x6 
	Eigen::Vector4d& attitudeQuat, 
	MatrixNd& covariance, 
	Eigen::Vector3d& gyroBias,
	Eigen::Vector3d& gyroMeas,
	Eigen::Vector3d& magMeas, 
	Eigen::Vector3d& magField,
	VectorNd& error
) {
/*	Input descriptions: gyroDt is the gyro sampling interval, sigma_bias is the
	gyro bias standard deviation, sigma_noise is the gyro noise standard 
	deviation sigma_mag is the magnetometer noise standard deviation, noiseCov
	is the noise covariance, attitudeQuat is the initial orientation quaternion,
	covariance is the estimated initial covariance, gyroBias is the estimated
	initial gyro bias, gyroMeas and magMeas are gyroscope and magnetometer 
	measurements, magField is the government provided expected magnetic field 
	vector at our location (we can calculate this on the fly or pre-load onto 
	the rocket). error is the estimated error vector.*/
// TODO: what exactly is error

/*	Fine tuning parameters described in the research paper. Recommmended values
	are a=1, lambda=1, and f=4 */ 
    const int a = 1;
    const int lambda = 1;
    const int f = 4;
	
/*	Half the number of sampling points for the extended Kalman filter. Should be
	n=6 unless you are modifying the algorithm */

	/* runFilter.m: 167 */
	//sigmaQuats.m: 1-33
	Eigen::Matrix<double, __N__, 2 * __N__ + 1> chi = chiValues(lambda, covariance, noiseCov, error);
	//sigmaQuats.m: 34-70
	Eigen::Matrix<double, 4, 2 * __N__ + 1> possQuats = quatDistribution(a,f,chi,attitudeQuat);

	//runFilter.m: 175 (Eq. 35)
	Eigen::Matrix<double, 3, 2 * __N__ + 1> possAngV = sigmaOmegas(gyroMeas, chi);
	//runFilter.m: 183 (Eq. 34)
	Eigen::Matrix<double, 4, 2 * __N__ + 1> possNewQuats = quatPropagate(possQuats,possAngV,gyroDt);
	//runFilter.m: 189
	Eigen::Matrix<double, __N__, 2 * __N__ + 1> possNewError = newChis(possNewQuats, chi, f, a);
	//predictError.m: 18
	VectorNd predError = predictError(lambda, possNewError, noiseCov);
	//predictError.m: 19-27
	MatrixNd predCov = predictCov(lambda, possNewError, noiseCov, 
		predError);
	//runFilter.m: 213
	Eigen::Matrix<double, 3, 2 * __N__ + 1> possExpMagMeas = sigmaMeas(possNewQuats, magField);
	
	//runFilter.m: 217
	Eigen::Vector3d predMagMeas = predictMeas(lambda, possExpMagMeas);

	//runFilter.m: 224
	Eigen::Matrix3d newInnovationCov = innovationCov(possExpMagMeas, 
		predMagMeas, lambda, sigma_mag);
	
	//runFilter.m: 229
	Eigen::Matrix<double, __N__, 3> newCrossCorrelation = crossCorr(possNewError, predError,
		possExpMagMeas, predMagMeas, lambda);

	//updateError.m: 27
	Eigen::Vector3d innovation = magMeas - predMagMeas;
	//updateError.m: 31
	Eigen::Matrix<double, __N__, 3> gain = newCrossCorrelation * 
		newInnovationCov.inverse();
	//runFilter.m: 257-258
	error = predError + gain*innovation; // Generate new error

	//updateError.m: 41
	covariance = predCov - gain * newInnovationCov * gain.transpose(); // New cov
	//runFilter.m: 251
	attitudeQuat = quatUpdate(error, f, a, possNewQuats); // new attitudeQuat
	//runFilter.m: 266
	error.head(3) << 0,0,0;
	

	return;
}



