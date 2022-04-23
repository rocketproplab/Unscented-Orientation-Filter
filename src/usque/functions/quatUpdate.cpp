#include "Eigen/Cholesky"




Eigen::Vector4d quatUpdate(
	Eigen::VectorNd& error, 
	const int f, 
	const int a, 
	Eigen::MatrixNd& possNewQuats
) {
	Eigen::Vector4d update;
	double fourVal = (-a * error.head(3).squaredNorm() + f * sqrt(f*f + 
		(1- a * a) * error.head(3).squaredNorm())) / (f * f + 
		error.head(3).squaredNorm());
	update.head(3) << ((a + fourVal) / f)*error.head(3);

	return kalmanArrayMult(update,possNewQuats.col(2 * __N__));
}