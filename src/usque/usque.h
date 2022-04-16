#ifndef USQUE_H
#define USQUE_H
#include <vector>
#include "Eigen/Cholesky"


#define N 6

Eigen::Matrix3d attitudeMatrix(Eigen::Vector4d& quat);

Eigen::MatrixXd chiValues(
	int lambda, 
	Eigen::MatrixXd& covariance, 
	Eigen::MatrixXd& noiseCov, 
	Eigen::VectorXd& error
	);

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

Eigen::Vector4d kalmanArrayInv(Eigen::Vector4d quat);

Eigen::Vector4d kalmanArrayMult(
	Eigen::Vector4d vecLeft, 
	Eigen::Vector4d vecRight
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

Eigen::MatrixXd sigmaMeas(
	Eigen::MatrixXd& possNewQuats, 
	Eigen::Vector3d& magField
);

Eigen::MatrixXd sigmaOmegas(
	Eigen::Vector3d gyroMeas, 
	Eigen::VectorXd error, 
	Eigen::MatrixXd chi
);


#endif