#include "usque.hpp"

Eigen::Vector4d quatUpdate(
	VectorNd& error, 
	const int f, 
	const int a, 
	Eigen::Matrix<double, 4, 2 * __N__ + 1>& possNewQuats
) {
	Eigen::Vector4d update;
	double fourVal = (-a * error.head(3).squaredNorm() + f * sqrt(f*f + 
		(1- a * a) * error.head(3).squaredNorm())) / (f * f + 
		error.head(3).squaredNorm());
	update.head(3) << ((a + fourVal) / f)*error.head(3);

	return kalmanArrayMult(update,possNewQuats.col(2 * __N__));
}