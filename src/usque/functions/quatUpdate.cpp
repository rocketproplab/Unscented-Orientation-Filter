#include "usque.hpp"

namespace RPL {
namespace USQUE {

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

}
}