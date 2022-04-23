#include "usque.hpp"

Eigen::Vector3d predictMeas(
	int lambda, 
	Eigen::Matrix<double, 3, 2 * __N__ + 1>& possExpMagMeas
) {
	Eigen::Vector3d predMagMeas;
	predMagMeas << (lambda * possExpMagMeas.col(2 * __N__) + 
		0.5*possExpMagMeas.leftCols(2 * __N__).rowwise().sum())/(__N__ + lambda);
	return predMagMeas;
}