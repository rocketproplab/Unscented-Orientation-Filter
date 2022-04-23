#include "usque.h"
#include <cstdlib>
#include <iostream>


Eigen::Matrix3d attitudeMatrix(Eigen::Vector4d& quat) {
	Eigen::Matrix<double, 3, 4>& eps;
	Eigen::Matrix<double, 4, 3>& psi;
	/*
	 * Block matrix:
	 * [
	 *   I * crossMatrix(quat[0:2]) 
	 *   -1 * quat[0:2].transpose()
	 * ]
	 */
	eps <<  quat(3),  quat(2), -quat(1), -quat(0),
	       -quat(2),  quat(3),  quat(0), -quat(1),
		    quat(1), -quat(0),  quat(3), -quat(2);

	psi <<  quat(3), -quat(2),  quat(1),
	        quat(2),  quat(3), -quat(0),
		   -quat(1),  quat(0),  quat(3),
		   -quat(0), -quat(1), -quat(2);
	result = eps * psi;
}

/* The filterStep() function is one iteration of the runFilter() function in runFilter.m. */

// JNIEXPORT void JNICALL
// Java_USQUE_filterStep(JNIEnv *env, jclass obj, jstring json_params) {
void filterStep(
	const double gyroDt, 
	const double sigma_bias, 
	const double sigma_noise, 
	const double sigma_mag, 
	Eigen::MatrixXd& noiseCov, //6x6 
	Eigen::Vector4d& attitudeQuat, 
	Eigen::MatrixXd& covariance, 
	Eigen::Vector3d& gyroBias,
	Eigen::Vector3d& gyroMeas,
	Eigen::Vector3d& magMeas, 
	Eigen::Vector3d& magField,
	Eigen::VectorXd& error
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
	Eigen::Matrix<double, __N__, 2 * __N__ + 1> chi = chiValues(lambda,covariance,noiseCov,error);
	//sigmaQuats.m: 34-70
	Eigen::MatrixXd possQuats = quatDistribution(a,f,chi,attitudeQuat);

	//runFilter.m: 175 (Eq. 35)
	Eigen::Matrix<double, 3, 2 * __N__ + 1> possAngV = sigmaOmegas(gyroMeas, chi);
	//runFilter.m: 183 (Eq. 34)
	Eigen::Matrix<double, 4, 2 * __N__ + 1> possNewQuats = quatPropagate(possQuats,possAngV,gyroDt);
	//runFilter.m: 189
	Eigen::Matrix<double, __N__, 2 * __N__ + 1> possNewError = newChis(possNewQuats, chi, f, a);
	//predictError.m: 18
	Eigen::VectorNd predError = predictError(lambda, possNewError, noiseCov);
	//predictError.m: 19-27
	Eigen::MatrixNd predCov = predictCov(lambda, possNewError, noiseCov, 
		predError);
	//runFilter.m: 213
	Eigen::MatrixXd possExpMagMeas = sigmaMeas(possNewQuats, magField);
	
	//runFilter.m: 217
	Eigen::Vector3d predMagMeas = predictMeas(lambda, possExpMagMeas);

	//runFilter.m: 224
	Eigen::MatrixXd newInnovationCov = innovationCov(possExpMagMeas, 
		predMagMeas, lambda, sigma_mag);
	
	//runFilter.m: 229
	Eigen::MatrixXd newCrossCorrelation = crossCorr(possNewError, predError,
		possExpMagMeas, predMagMeas, lambda);

	//updateError.m: 27
	Eigen::Vector3d innovation = magMeas - predMagMeas;
	//updateError.m: 31
	Eigen::MatrixXd gain = newCrossCorrelation * 
		newInnovationCov.inverse();
	//runFilter.m: 257-258
	error = predError + gain*innovation; // Generate new error

	//updateError.m: 41
	covariance = predCov - gain*newInnovationCov * gain.transpose(); // New cov
	//runFilter.m: 251
	attitudeQuat = quatUpdate(error, f, a, possNewQuats); // new attitudeQuat
	//runFilter.m: 266
	error.head(3) << 0,0,0;
	

	return;
}




/*
	Corresponds to sigmaQuats:1-33
 */
Eigen::MatrixXd chiValues(int lambda, Eigen::MatrixXd covariance, 
	Eigen::MatrixXd noiseCov, Eigen::VectorXd error) {
	using namespace Eigen;
	const int n = 6;
	MatrixXd cholInput = (n+lambda)*(covariance+noiseCov);

	// take the Cholesky decomposition such that cholInput = sigma * 
	//		transpose(sigma)
	// same as in the paper, should not require transposing output as in matlab
	MatrixXd sigma = cholInput.llt().matrixL();
	MatrixXd chi(n,2*n+1);
	MatrixXd zeros = MatrixXd::Zero(n,1);
	chi << sigma, (-1*sigma), zeros;
	chi = chi + error.replicate<1,2*n+1>();
	return chi;
}

Eigen::MatrixXd quatDistribution(int a, int f, Eigen::MatrixXd chi, 
	Eigen::Vector4d attitudeQuat) {
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

Eigen::Vector4d kalmanArrayMult(Eigen::Vector4d vecLeft,Eigen::Vector4d 
	vecRight) {
	using namespace Eigen;
	MatrixXd eps(4,4);
	eps.block(0,0,3,3) << vecRight(3,0) * MatrixXd::Identity(3,3) + 
		crossMatrix(vecRight.block(0,0,3,1));
	eps.block(3,0,1,3) << -1*vecRight.block(0,0,3,1).transpose();
	eps.col(3) << vecRight;
	Vector4d prodVec = eps*vecLeft;
	return prodVec;
}

Eigen::MatrixXd crossMatrix(Eigen::Vector3d vec) {
	using namespace Eigen;
	MatrixXd mat(3,3);
	// cerr << "jesus" << endl;
	mat << 0, -1*vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
	return mat;
}

Eigen::MatrixXd sigmaOmegas(Eigen::Vector3d gyroMeas, Eigen::VectorXd error, 
	Eigen::MatrixXd chi) {
	using namespace Eigen;
	const int n = 6;
	// cerr << "entered sigmaOmegas" << endl;
	MatrixXd possAngV(3,2*n+1);
	possAngV << gyroMeas.replicate<1,2*n+1>() - chi.bottomRows(3);
	return possAngV;
}

	
Eigen::MatrixXd newChis(
	Eigen::MatrixXd possNewQuats, 
	Eigen::MatrixXd chi,
	int f, 
	int a
) {
	using namespace Eigen;
	const int n = 6;
	MatrixXd dqK1(4,2*n+1);
	for(int i = 0; i < 2*n+1; i++) {
		dqK1.col(i) << kalmanArrayMult(possNewQuats.col(i), kalmanArrayInv(
			possNewQuats.col(2*n)));
	}
	// cerr << "dqK1 good" << endl;
	MatrixXd possNewError(6,2*n+1);
	for(int i = 0; i < 2*n; i++) {
		possNewError.block(0,i,3,1) << f*dqK1.block(0,i,3,1)/(a+dqK1(3,i));
	}
	// cerr << "step1 good" << endl;

	possNewError.block(0,2*n,3,1) << 0,0,0;
	possNewError.bottomRows(3) <<  chi.bottomRows(3);
	return possNewError;
}

Eigen::Vector4d kalmanArrayInv(Eigen::Vector4d quat) {
	using namespace Eigen;
	// cerr << "enter kalmanArrayInv" << endl;
	Vector4d invQuat;
	invQuat << -quat.head(3), quat.squaredNorm();
	return invQuat;
}
	
Eigen::VectorXd predictError(int lambda, Eigen::MatrixXd possNewError, 
	Eigen::MatrixXd noiseCov) {
	using namespace Eigen;
	const int n = 6;
	VectorXd predError(6);
	predError << (lambda * possNewError.col(2*n) + 0.5*possNewError.
		leftCols(2*n).rowwise().sum())/(n+lambda);
	// cerr << "predError: " << endl << predError << endl;
	return predError;
}

Eigen::MatrixXd predictCov(int lambda, Eigen::MatrixXd possNewError, 
	Eigen::MatrixXd noiseCov, Eigen::VectorXd predError) {
	using namespace Eigen;
	const int n = 6;
	MatrixXd intermediate(6,2*n);
	intermediate = possNewError.leftCols(2*n) - predError.replicate<1,2*n>();
	MatrixXd predCov(6,6);
	predCov << (lambda*(possNewError.col(2*n) - predError)*(possNewError.
		col(2*n) - predError).transpose() + 0.5*intermediate*intermediate.
		transpose())/(n + lambda) + noiseCov;
	return predCov;
}

Eigen::MatrixXd sigmaMeas(Eigen::MatrixXd& possNewQuats, Eigen::Vector3d& magField, Eigen::Matrix<3, 2 * N + 1>& result) {
	for(int i = 0; i < 2*N; i++) {
		possExpMagMeas.col(i) = attitudeMatrix(possNewQuats.col(i))*magField;
	}
	return possExpMagMeas;
}
	

Eigen::Vector3d predictMeas(int lambda, Eigen::MatrixXd possExpMagMeas) {
	using namespace Eigen;
	const int n = 6;
	Vector3d predMagMeas = (lambda*possExpMagMeas.col(2*n) + 
		0.5*possExpMagMeas.leftCols(2*n).rowwise().sum())/(n+lambda);
	return predMagMeas;
}

Eigen::MatrixXd innovationCov(Eigen::MatrixXd possExpMagMeas, 
	Eigen::Vector3d predMagMeas, int lambda, double sigma_mag) {
	using namespace Eigen;
	const int n = 6;
	MatrixXd covSum;
	covSum << lambda*(possExpMagMeas.col(2*n) - predMagMeas) * 
		(possExpMagMeas.col(2*n) - predMagMeas).transpose();
	
	for(int i = 0; i < 2*n; i++) {
		covSum = covSum + (possExpMagMeas.col(i) - predMagMeas) * 
			0.5*(possExpMagMeas.col(i) - predMagMeas).transpose();
	}

	return covSum/(n + lambda) + (sigma_mag*sigma_mag) * 
		MatrixXd::Identity(3,3);
}

Eigen::MatrixXd crossCorr(Eigen::MatrixXd possNewError,
	Eigen::VectorXd predError, Eigen::MatrixXd possExpMagMeas,
	Eigen::Vector3d predMagMeas, int lambda) {
	using namespace Eigen;
	const int n = 6;
	MatrixXd covSum;
	covSum << lambda*(possNewError.col(2*n) - predError) * 
		(possExpMagMeas.col(2*n) -predMagMeas).transpose();
	
	for(int i = 0; i < 2*n; i++) {
		covSum = covSum + (possNewError.col(i) - predError) * 
			0.5*(possExpMagMeas.col(i) - predMagMeas);
	}

	return covSum/(n+lambda);
}

Eigen::Vector4d quatUpdate(Eigen::VectorXd error, int f, int a, 
	Eigen::MatrixXd possNewQuats) {
	using namespace Eigen;
	const int n = 6;
	Vector4d update;
	double fourVal = (-a*error.head(3).squaredNorm() + f*sqrt(f*f + 
		(1-a*a)*error.head(3).squaredNorm()))/(f*f + 
		error.head(3).squaredNorm());
	update.head(3) << ((a+fourVal)/f)*error.head(3);

	return kalmanArrayMult(update,possNewQuats.col(2*n));
}




