#include "usque.hpp"

namespace RPL {
namespace USQUE {

Matrix_3x13d sigmaOmegas(
	Eigen::Vector3d& gyroMeas, 
	Matrix_6x13d& chi
) {
	// cerr << "entered sigmaOmegas" << endl;
	Matrix_3x13d possAngV;
	//Note: bottom 3 rows are the errors.
	possAngV << gyroMeas.replicate<1,2 * SIZE + 1>() - chi.bottomRows(3);
	return possAngV;

}

}
}