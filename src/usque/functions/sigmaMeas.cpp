#include "usque.hpp"

namespace RPL {
namespace USQUE {

Matrix_3x13d sigmaMeas(
	Matrix_4x13d& possNewQuats, 
	Eigen::Vector3d& magField 
) {
	Matrix_3x13d possExpMagMeas;
	for(int i = 0; i < 2 * SIZE + 1; i++) {
		possExpMagMeas.col(i) = attitudeMatrix(possNewQuats.col(i)) * magField;
	}
	return possExpMagMeas;
}

}
}