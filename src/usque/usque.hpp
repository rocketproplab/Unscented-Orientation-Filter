#ifndef USQUE_HPP
	#define USQUE_HPP
	#define SIZE 6
	#include "Eigen/Dense"
namespace Usque {
//Convenient typedefs

typedef Eigen::Matrix<double, SIZE, SIZE>         Matrix_6x6d;
typedef Eigen::Vector<double, SIZE>               Vector6d;
typedef Eigen::Matrix<double, SIZE, 2 * SIZE + 1> Matrix_6x13d; 
typedef Eigen::Matrix<double, 3, 2 * SIZE + 1>    Matrix_3x13d;
typedef Eigen::Matrix<double, 4, 2 * SIZE + 1>    Matrix_4x13d;
typedef Eigen::Matrix<double, 6, 3>               Matrix_6x3d;
typedef Eigen::Matrix<double, 3, 6>               Matrix_3x6d;
typedef Eigen::Matrix<double, 3, 4>               Matrix_3x4d;
typedef Eigen::Matrix<double, 4, 3>               Matrix_4x3d;
/*	
	Run a step of the algorithm. This is the workhorse of the library.

	gyroDt: gyro sampling interval
	sigma_bias:
Input descriptions: gyroDt is the gyro sampling interval, sigma_bias is the
	gyro bias standard deviation, sigma_noise is the gyro noise standard 
	deviation sigma_mag is the magnetometer noise standard deviation, noiseCov
	is the noise covariance, attitudeQuat is the initial orientation quaternion,
	covariance is the estimated initial covariance, gyroBias is the estimated
	initial gyro bias, gyroMeas and magMeas are gyroscope and magnetometer 
	measurements, magField is the government provided expected magnetic field 
	vector at our location (we can calculate this on the fly or pre-load onto 
	the rocket). error is the estimated error vector.*/
// TODO: what exactly is error
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
); 

/*
	Implemented in attitudeMatrix.cpp
	quat: 4-vector
	Returns: (3 * 3) matrix
 */
Eigen::Matrix3d attitudeMatrix(
	Eigen::Vector4d&     quat
);

/*
	r-value reference version
 */
Eigen::Matrix3d attitudeMatrix(
	Eigen::Vector4d&&    quat
);

/*
	Implemented in sigmaQuats.cpp
	lambda: constant scalar
	covariance: (N * N) matrix
	noiseCov: (N * N) matrix
	Returns: (N * (2N + 1)) matrix.
 */
Matrix_6x13d chiValues(
	const int            lambda, 
	Matrix_6x6d&         covariance, 
	Matrix_6x6d&         noiseCov, 
	Vector6d&            error
); 

/*
	
 */
Matrix_6x3d crossCorr(
	Matrix_6x13d&        possNewError, //6x13
	Vector6d&            predError, //6
	Matrix_3x13d&        possExpMagMeas, //3x13
	Eigen::Vector3d&     predMagMeas, //3
	const int            lambda
);

Eigen::Matrix3d crossMatrix(
	Eigen::Vector3d&     vec
);

Eigen::Matrix3d crossMatrix(
	Eigen::Vector3d&&    vec
);



/*  crossMatrix returns the cross product matrix of the given vector
    vec is a 3-vector */
// Eigen::MatrixXd crossMatrix(Eigen::Vector3d vec);

/*  innovationCov2.m
	innovationCov2(possExpMagMeas,predMagMeas,lambda,sigma_mag,n)
    This function calculates the innovation covariance
        Parameters:
    possExpMagMeas is the distribution of possible mag measurements based
      on the magnetic field at this location
    predMagMeas is the predicted magnetometer measurement at the location
    sigma_mag is the standard deviation of magnetometer noise
    lambda and n are constants*/
Eigen::Matrix3d innovationCov(
	Matrix_3x13d&        possExpMagMeas, 
	Eigen::Vector3d&     predMagMeas, 
	const int            lambda, 
	const double         sigma_mag
);

/*  kalmanArrayInv.m
    kalmanArrayInv finds the inverse of a quaternion
    quat is the quaternion whose inverse should be found
    This uses the quaternion convention with the scalar cosine term 4th*/
Eigen::Vector4d kalmanArrayInv(
	Eigen::Vector4d&     quat
);
Eigen::Vector4d kalmanArrayInv(
	Eigen::Vector4d&&    quat
);

/*  kalmanArrayMult multiplies two quaternions using
      the array method
    q and p are both input quaternions (column 4vecs)
      the fourth component is the scalar cosine one
    quat is the quaternion such that:
    quat = q*p*/
Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d&     vecLeft, 
	Eigen::Vector4d&     vecRight
);
Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d&&    vecLeft, 
	Eigen::Vector4d&     vecRight
);

Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d&     vecLeft, 
	Eigen::Vector4d&&    vecRight
);

Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d&&    vecLeft, 
	Eigen::Vector4d&&    vecRight
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
Matrix_6x13d newChis(
	Matrix_4x13d&        possNewQuats, 
	Matrix_6x13d&        chi,
	const int            f, 
	const int            a
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
Vector6d predictError(
	const int            lambda, 
	Matrix_6x13d&        possNewError, 
	Matrix_6x6d&         noiseCov
);

Matrix_6x6d predictCov(
	const int            lambda, 
	Matrix_6x13d&        possNewError, 
	Matrix_6x6d&         noiseCov, 
	Vector6d&            predError
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
	const int            lambda, 
	Matrix_3x13d&        possExpMagMeas
);

Matrix_4x13d quatDistribution(
	const int            a, 
	const int            f, 
	Matrix_6x13d&        chi, 
	Eigen::Vector4d&     attitudeQuat
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
Matrix_4x13d quatPropagate(
	Matrix_4x13d&        possQuats, 
	Matrix_3x13d&        possAngV, 
	const double         gyroDt
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
	Vector6d&            error, 
	const int            f, 
	const int            a, 
	Matrix_4x13d&        possNewQuats
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
Matrix_3x13d sigmaMeas(
	Matrix_4x13d&        possNewQuats, 
	Eigen::Vector3d&     magField 
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
Matrix_3x13d sigmaOmegas(
	Eigen::Vector3d&     gyroMeas, 
	Matrix_6x13d&        chi
);

}
#endif