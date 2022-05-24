#include "usque.hpp"

namespace Usque {

/*
 * attitudeMatrix.m
 * 
 */
Eigen::Matrix3d attitudeMatrix(Eigen::Vector4d& quat) {
	Matrix_3x4d eps;
	Matrix_4x3d psi;
	//Eq. 16a, expanded.
	eps <<  quat(3),  quat(2), -quat(1), -quat(0),
	       -quat(2),  quat(3),  quat(0), -quat(1),
		    quat(1), -quat(0),  quat(3), -quat(2);

	//Eq. 16b, expanded.
	psi <<  quat(3), -quat(2),  quat(1),
	        quat(2),  quat(3), -quat(0),
		   -quat(1),  quat(0),  quat(3),
		   -quat(0), -quat(1), -quat(2);
	return eps * psi;
}

Eigen::Matrix3d attitudeMatrix(
	Eigen::Vector4d&&    quat
) {
	return attitudeMatrix(quat);
}

/*
 * crossCorr.m
 */
Matrix_6x3d crossCorr(
	Matrix_6x13d& possNewError, //6x13
    Vector6d& predError, //6
    Matrix_3x13d& possExpMagMeas, //3x13
    Eigen::Vector3d& predMagMeas, //3
    const int lambda
) {
	Matrix_6x3d covSum;
	covSum << lambda * (possNewError.col(2 * SIZE) - predError) * 
		(possExpMagMeas.col(2 * SIZE) - predMagMeas).transpose();
	
	for(int i = 0; i < 2 * SIZE; i++) {
		covSum = covSum + (possNewError.col(i) - predError) * 
			0.5*(possExpMagMeas.col(i) - predMagMeas).transpose();
	}

	return covSum/(SIZE + lambda);
}

Eigen::Matrix3d crossMatrix(
	Eigen::Vector3d&     vec
) {
	Eigen::Matrix3d result;
	result <<         0, -vec(2),  vec(1),
	           vec(2),         0, -vec(0), 
			  -vec(1),  vec(0),         0;
	return result;
}

Eigen::Matrix3d crossMatrix(
	Eigen::Vector3d&&    vec
) {
	return crossMatrix(vec);
}

/*
 * innovationCov.m
 */
Eigen::Matrix3d innovationCov(
    Eigen::Matrix<double, 3, 2 * SIZE + 1>& possExpMagMeas, 
	Eigen::Vector3d& predMagMeas,
    int lambda,
    double sigma_mag
) {
	Eigen::Matrix3d covSum;
	covSum << lambda*(possExpMagMeas.col(2 * SIZE) - predMagMeas) * 
		(possExpMagMeas.col(2 * SIZE) - predMagMeas).transpose();
	
	for(int i = 0; i < 2*SIZE; i++) {
		covSum = covSum + (possExpMagMeas.col(i) - predMagMeas) * 
			0.5*(possExpMagMeas.col(i) - predMagMeas).transpose();
	}

	return covSum / (SIZE + lambda) + (sigma_mag*sigma_mag) * 
		Eigen::Matrix3d::Identity();
}


Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d& vecLeft,
	Eigen::Vector4d& vecRight
) {
	Eigen::Matrix4d eps;
	eps.block(0,0,3,3) << vecRight(3) * Eigen::Matrix3d::Identity() + 
		crossMatrix(1 * vecRight.head<3>());
	eps.block(3,0,1,3) << -1*vecRight.block(0,0,3,1).transpose();
	eps.col(3) << vecRight;
	Eigen::Vector4d prodVec = eps*vecLeft;
	return prodVec;
}

Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d& vecLeft,
	Eigen::Vector4d&& vecRight
) {
	return kalmanArrayMult(vecLeft, vecRight);
}

Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d&& vecLeft,
	Eigen::Vector4d& vecRight
) {
	return kalmanArrayMult(vecLeft, vecRight);
}

Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d&& vecLeft,
	Eigen::Vector4d&& vecRight
) {
	return kalmanArrayMult(vecLeft, vecRight);
}

Eigen::Vector4d kalmanArrayInv(
	Eigen::Vector4d&     quat
) {
	Eigen::Vector4d inverse;
	inverse << -quat.head<3>(), quat(3);
	double norm = quat.norm();
	inverse = inverse / (norm * norm);
	return inverse;
}

Eigen::Vector4d kalmanArrayInv(
	Eigen::Vector4d&&    quat
) {
	return kalmanArrayInv(quat);
}

/*
 * newChis.m
 */
Matrix_6x13d newChis(
	Matrix_4x13d&        possNewQuats, 
	Matrix_6x13d&        chi,
	const int            f, 
	const int            a
) {
	Matrix_4x13d dqK1;
	for(int i = 0; i < 2 * SIZE + 1; i++) {
		dqK1.col(i) << kalmanArrayMult(possNewQuats.col(i), kalmanArrayInv(
			possNewQuats.col(2 * SIZE)));
	}
	// cerr << "dqK1 good" << endl;
	Matrix_6x13d possNewError;
	for(int i = 0; i < 2*SIZE; i++) {
		possNewError.block(0,i,3,1) << f*dqK1.block(0,i,3,1)/(a+dqK1(3,i));
	}
	// cerr << "step1 good" << endl;

	possNewError.block(0,2*SIZE,3,1) << 0,0,0;
	possNewError.bottomRows(3) <<  chi.bottomRows(3);
	return possNewError;
}


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

/*
 * predictMeas.m
 */
Eigen::Vector3d predictMeas(
	int lambda, 
	Matrix_3x13d& possExpMagMeas
) {
	Eigen::Vector3d predMagMeas;
	predMagMeas << (lambda * possExpMagMeas.col(2 * SIZE) + 
		0.5*possExpMagMeas.leftCols(2 * SIZE).rowwise().sum())/(SIZE + lambda);
	return predMagMeas;
}

/*
 * quatPropagate.m
 */
Matrix_4x13d quatPropagate(
	Matrix_4x13d& possQuats, 
	Matrix_3x13d& possAngV, 
	double gyroDt
) {
	Matrix_4x13d possNewQuats;
	for(int i = 0; i < 2 * SIZE + 1; i++) {
		Eigen::Vector3d psiK = sin(0.5*gyroDt*possAngV.col(i).norm())*
			possAngV.col(i)/possAngV.col(i).norm();
		Eigen::Matrix4d omega;
		omega.block(0,0,3,3) << cos(0.5*gyroDt*possAngV.col(i).norm())*
			Eigen::Matrix3d::Identity() - crossMatrix(psiK);
		omega.block(0,3,3,1) << psiK;
		omega.row(3) << -1*psiK.transpose(), cos(0.5*gyroDt*possAngV.col(i).
			norm());
		possNewQuats.col(i) << omega*possQuats.col(i);
	}
	return possNewQuats;
}

/*
 * quatUpdate.m
 */
Eigen::Vector4d quatUpdate(
	Vector6d& error, 
	const int f, 
	const int a, 
	Matrix_4x13d& possNewQuats
) {
	Eigen::Vector4d update;
	double fourVal = (-a * error.head(3).squaredNorm() + f * sqrt(f*f + 
		(1- a * a) * error.head(3).squaredNorm())) / (f * f + 
		error.head(3).squaredNorm());
	update.head(3) << ((a + fourVal) / f)*error.head(3);

	return kalmanArrayMult(update,possNewQuats.col(2 * SIZE));
}

/*
 * sigmaMeas.m
 */
Matrix_3x13d sigmaMeas(
	Matrix_4x13d& possNewQuats, 
	Eigen::Vector3d& magField 
) {
	Matrix_3x13d possExpMagMeas;
	for(int i = 0; i < 2 * SIZE + 1; i++) {
		possExpMagMeas.col(i) = attitudeMatrix(possNewQuats.col(i)) * magField;
	}
	return possExpMagMeas;
}

/*
 * sigmaOmegas.m
 */
Matrix_3x13d sigmaOmegas(
	Eigen::Vector3d& gyroMeas, 
	Matrix_6x13d& chi
) {
	// cerr << "entered sigmaOmegas" << endl;
	Matrix_3x13d possAngV;
	//Note: bottom 3 rows are the errors.
	possAngV << gyroMeas.replicate<1,2 * SIZE + 1>() - chi.bottomRows(3);
	return possAngV;

}

/*
	Corresponds to sigmaQuats.m:1-33
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

/*
 * sigmaQuats.m
 */
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

/*
 * runFilter.m
 */
void filterStep(
	const int        a,
	const int        lambda,
	const int        f,
	const double     gyroDt, 
	const double     sigmaBias, 
	const double     sigmaNoise, 
	const double     sigmaMag, 
	Matrix_6x6d&     noiseCov, 
	Eigen::Vector4d& attitudeQuat, 
	Matrix_6x6d&     covariance, 
	Eigen::Vector3d& gyroMeas,
	Eigen::Vector3d& magMeas, 
	Eigen::Vector3d& magField,
	Vector6d& error
) {

	
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
		predMagMeas, lambda, sigmaMag).inverse();

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
	

}

}