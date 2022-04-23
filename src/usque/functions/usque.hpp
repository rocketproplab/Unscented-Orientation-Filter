#ifndef USQUE_H
	#define USQUE_H
	#define __N__ 6
	#include "Eigen/Cholesky"

//Matrix is N x N, or 6x6
typedef Eigen::Matrix<double, __N__, __N__> MatrixNd;
//Vector is size 6
typedef Eigen::Vector<double, __N__> VectorNd;
//Matrix is (N x (2 * N + 1))
typedef Eigen::Matrix<double, __N__, 2 * __N__ + 1> MatrixErr6; 
/*
	Implemented in attitudeMatrix.cpp
	quat: 4-vector
	Returns: (3 * 3) matrix
 */
Eigen::Matrix3d attitudeMatrix(
	Eigen::Vector4d& quat
);

Eigen::Matrix3d attitudeMatrix(
	Eigen::Vector4d&& quat
);

/*
	Implemented in sigmaQuats.cpp
	lambda: constant scalar
	covariance: (N * N) matrix
	noiseCov: (N * N) matrix
	Returns: (N * (2N + 1)) matrix.
 */
MatrixErr6 chiValues(
	const int lambda, 
	MatrixNd& covariance, 
	MatrixNd& noiseCov, 
	VectorNd& error
); 

/*
	
 */
Eigen::Matrix<double, 6, 3> crossCorr(
	MatrixErr6& possNewError, //6x13
	VectorNd& predError, //6
	Eigen::Matrix<double, 3, 2 * __N__ + 1>& possExpMagMeas, //3x13
	Eigen::Vector3d& predMagMeas, //3
	const int lambda
);

Eigen::Matrix3d crossMatrix(Eigen::Vector3d& vec);

Eigen::Matrix3d crossMatrix(Eigen::Vector3d&& vec);
/*  crossMatrix returns the cross product matrix of the given vector
    vec is a 3-vector */
Eigen::MatrixXd crossMatrix(Eigen::Vector3d vec);

/*  innovationCov2.m
	innovationCov2(possExpMagMeas,predMagMeas,lambda,sigma_mag,n)
    This function calculates the innovation covariance
        Parameters:
    possExpMagMeas is the distribution of possible mag measurements based
      on the magnetic field at this location
    predMagMeas is the predicted magnetometer measurement at the location
    sigma_mag is the standard deviation of magnetometer noise
    lambda and n are constants*/
Eigen::MatrixXd innovationCov(
	Eigen::MatrixXd possExpMagMeas, 
	Eigen::Vector3d predMagMeas, 
	int lambda, 
	double sigma_mag
);

/*  kalmanArrayInv.m
    kalmanArrayInv finds the inverse of a quaternion
    quat is the quaternion whose inverse should be found
    This uses the quaternion convention with the scalar cosine term 4th*/
Eigen::Vector4d kalmanArrayInv(
	Eigen::Vector4d& quat
);
Eigen::Vector4d kalmanArrayInv(
	Eigen::Vector4d&& quat
);

/*  kalmanArrayMult multiplies two quaternions using
      the array method
    q and p are both input quaternions (column 4vecs)
      the fourth component is the scalar cosine one
    quat is the quaternion such that:
    quat = q*p*/
Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d& vecLeft, 
	Eigen::Vector4d& vecRight
);
Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d&& vecLeft, 
	Eigen::Vector4d& vecRight
);

Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d& vecLeft, 
	Eigen::Vector4d&& vecRight
);

Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d&& vecLeft, 
	Eigen::Vector4d&& vecRight
);

/*  newChis.m
	newChis(possNewQuats,possBias,n,f,a)
    finds possible new errors in the angular velocity measurements
        Parameters:
    possNewQuats are the possible orientations of the spacecraft after
      the most recent rotation measured by the gyro
    possBias are possible bias values in the gyro measurement
    n,f,a are constants and fine tuning values
        Result:
    possNewError are the possible new attitude error vectors*/
Eigen::Matrix<double, __N__, 2 * __N__ + 1> newChis(
	Eigen::Matrix<double, 4, 2 * __N__ + 1>& possNewQuats, 
	Eigen::Matrix<double, __N__, 2 * __N__ + 1>& chi,
	const int f, 
	const int a
); 

/*  predictError(lambda,possNewError,noiseCov,n)
    predict the gyro measurement error and new covariance for this step.
        Parameters:
    possNewError are the possible new attitude error vectors
    noiseCov is the constant noise covariance QbarK
    lambda and n are a fine tuning parameter and a constant
        Results:
    predError is the predicted gyro error vector
    predCov is the predicted new covariance*/
VectorNd predictError(
	const int lambda, 
	Eigen::Matrix<double, __N__, 2 * __N__ + 1>& possNewError, 
	MatrixNd& noiseCov
);

MatrixNd predictCov(
	const int lambda, 
	Eigen::Matrix<double, __N__, 2 * __N__ + 1>& possNewError, 
	MatrixNd& noiseCov, 
	VectorNd& predError
);

/*  predictMeas.m
	predictMeas(lambda,possExpMagMeas,n)
        predict the measurement the magnetometer will make based on the
      possible magnetometer measurements
    Parameters:
    possExpMagMeas are the possible magnetometer measurements that you 
      expect
    lambda and n are fine tuning and a constant
        Results:
    predMagMeas is the predicted magnetometer measurement*/
Eigen::Vector3d predictMeas(
	int lambda, 
	Eigen::MatrixXd possExpMagMeas
);

Eigen::Matrix<double, 4, 2 * __N__ + 1> quatDistribution(
	int a, 
	int f, 
	MatrixNd& chi, 
	Eigen::Vector4d& attitudeQuat
);

/*  quatPropagate.m
	quatPropagate(possQuats,possAngV,gyroDt)
    finds the possible orientations of the spacecraft after the possible
      rotations of the spacecraft
        Parameters:
    possQuats is the array of pre-propogation sigma quaternions
    possAngV is the array of estimated sigma angular velocities (measured
	minus Chi bias sigma vectors)
    gyroDt is the sampling interval in the gyro
        Results:
    possNewQuats is the array of propagated sigma quaternions */
Eigen::Matrix<double, 4, 2 * __N__ + 1> quatPropagate(
	Eigen::Matrix<double, 4, 2 * __N__ + 1>& possQuats, 
	Eigen::Matrix<double, 3, 2 * __N__ + 1>& possAngV, 
	double gyroDt
);

/*  quatUpdate.m
	quatUpdate(error,f,a,possNewQuats,n)
    updates the orientation quaternion by removing the estimated error
      from the propogated previous orientation.
    
    Parameters:
    error is the approximated error vector of this orientation
      calculation
    possNewQuats are the possible propagated orientation quaternions
    f and a are fine tuning constants
    n is a constant
        Results:
    attitudeQuat is the best estimate of the current spacecraft 
      orientation.*/
Eigen::Vector4d quatUpdate(
	VectorNd& error, 
	const int f, 
	const int a, 
	Eigen::Matrix<double, 4, 2 * __N__ + 1>& possNewQuats
);

/*  sigmaMeas.m
	sigmaMeas(possNewQuats,magField,n)
        Parameters:
    possNewQuats are the possible orientation quaternions after the most
      recent rotation
    magField is the magnetic field vector at this location from the World
      Magnetic Model
    n is a constant
        Results:
    possExpMagMeas are the possible expected magnetometer measurements.
      They represent the distribution of magnetometer measurements we
      think we can get based on what we think is the magnetic field at
      this location and our possible orientations after the last
      rotation. */
Eigen::Matrix<double, 3, 2 * __N__ + 1> sigmaMeas(
	Eigen::Matrix3d& possNewQuats, 
	Eigen::Vector3d& magField 
);

// sigmaOmegas.m
/* sigmaOmegas(gyroMeas,error,possBias,n)
    finds possible true angular velocities of the spacecraft based on the
      measured angular velocity and the possible true orientations
        Parameters:
    gyroMeas is the angular velocity measured by the spacecraft
    error is the best estimate of the error in gyroMeas compared to the
      true angular velocity
    possBias are possible bias values in gyroMeas compared to the true
      angular velocity
    n is a constant representing half of (the number of possible values
      -1)
        Results:
    possAngV is the possible true angular velocities*/
Eigen::Matrix<double, 3, 2 * __N__ + 1> sigmaOmegas(
	Eigen::Vector3d& gyroMeas, 
	MatrixErr6& chi
);

#endif