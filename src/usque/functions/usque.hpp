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
Eigen::MatrixXd crossCorr(
	Eigen::MatrixXd possNewError,
	Eigen::VectorXd predError, 
	Eigen::MatrixXd possExpMagMeas,
	Eigen::Vector3d predMagMeas, 
	int lambda
);

Eigen::MatrixXd crossMatrix(Eigen::Vector3d vec);

Eigen::MatrixXd innovationCov(
	Eigen::MatrixXd possExpMagMeas, 
	Eigen::Vector3d predMagMeas, 
	int lambda, 
	double sigma_mag
);

Eigen::Vector4d kalmanArrayInv(
	Eigen::Vector4d&& quat
);

Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d&& vecLeft, 
	Eigen::Vector4d& vecRight
);

Eigen::MatrixXd newChis(
	Eigen::MatrixXd possNewQuats, 
	Eigen::MatrixXd chi, 
	int f, 
	int a
);

Eigen::VectorXd predictError(
	int lambda, 
	Eigen::MatrixXd possNewError, 
	Eigen::MatrixXd noiseCov
);
Eigen::MatrixXd predictCov(
	int lambda, 
	Eigen::MatrixXd possNewError, 
	Eigen::MatrixXd noiseCov, 
	Eigen::VectorXd predError
);

Eigen::Vector3d predictMeas(
	int lambda, 
	Eigen::MatrixXd possExpMagMeas
);

Eigen::MatrixXd quatDistribution(
	int a, 
	int f, 
	Eigen::MatrixXd chi, 
	Eigen::Vector4d attitudeQuat
);
Eigen::MatrixXd quatPropagate(
	Eigen::MatrixXd possQuats, 
	Eigen::MatrixXd possAngV, 
	double gyroDt
);

Eigen::Vector4d quatUpdate(
	Eigen::VectorXd error, 
	int f, 
	int a, 
	Eigen::MatrixXd possNewQuats
);

Eigen::Matrix<double, 3, 2 * __N__ + 1> sigmaMeas(
	Eigen::Matrix3d& possNewQuats, 
	Eigen::Vector3d& magField 
);

Eigen::MatrixXd sigmaOmegas(
	Eigen::Vector3d gyroMeas, 
	Eigen::VectorXd error, 
	Eigen::MatrixXd chi
);

#endif