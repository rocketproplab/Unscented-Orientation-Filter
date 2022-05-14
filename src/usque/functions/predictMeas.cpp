#include "usque.hpp"

namespace RPL {
namespace USQUE {

Eigen::Vector3d predictMeas(
	int lambda, 
	Matrix_3x13d& possExpMagMeas
) {
	Eigen::Vector3d predMagMeas;
	predMagMeas << (lambda * possExpMagMeas.col(2 * SIZE) + 
		0.5*possExpMagMeas.leftCols(2 * SIZE).rowwise().sum())/(SIZE + lambda);
	return predMagMeas;
}

}
}