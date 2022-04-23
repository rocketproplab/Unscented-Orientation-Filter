#include "usque.hpp"

MatrixErr6 crossCorr(
	MatrixErr6& possNewError,
	VectorNd& predError, 
	Eigen::Matrix<double, __N__, 2 * __N__ + 1>& possExpMagMeas,
	VectorNd& predMagMeas, 
	const int lambda
) {
	MatrixErr6 covSum;
	covSum << lambda * (possNewError.col(2 * __N__) - predError) * 
		(possExpMagMeas.col(2 * __N__) - predMagMeas).transpose();
	
	for(int i = 0; i < 2 * __N__; i++) {
		covSum = covSum + (possNewError.col(i) - predError) * 
			0.5*(possExpMagMeas.col(i) - predMagMeas).transpose();
	}

	return covSum/(__N__ + lambda);
}