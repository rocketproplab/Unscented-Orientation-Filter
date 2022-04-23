#include "usque.hpp"

Eigen::Vector3d predictMeas(int lambda, Eigen::MatrixXd possExpMagMeas) {
	using namespace Eigen;
	const int n = 6;
	Vector3d predMagMeas = (lambda*possExpMagMeas.col(2*n) + 
		0.5*possExpMagMeas.leftCols(2*n).rowwise().sum())/(n+lambda);
	return predMagMeas;
}