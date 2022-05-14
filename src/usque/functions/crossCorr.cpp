#include "usque.hpp"

namespace RPL {
namespace USQUE {

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

}
}