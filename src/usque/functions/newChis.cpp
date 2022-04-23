#include "usque.hpp"

Eigen::Matrix<double, __N__, 2 * __N__ + 1> newChis(
	Eigen::Matrix<double, 4, 2 * __N__ + 1>& possNewQuats, 
	Eigen::Matrix<double, __N__, 2 * __N__ + 1>& chi,
	const int f, 
	const int a
) {
	Eigen::Matrix<double, 4, 2 * __N__ + 1> dqK1;
	for(int i = 0; i < 2 * __N__ + 1; i++) {
		dqK1.col(i) << kalmanArrayMult(possNewQuats.col(i), kalmanArrayInv(
			possNewQuats.col(2 * __N__)));
	}
	// cerr << "dqK1 good" << endl;
	Eigen::Matrix<double, __N__, 2 * __N__ + 1> possNewError;
	for(int i = 0; i < 2*__N__; i++) {
		possNewError.block(0,i,3,1) << f*dqK1.block(0,i,3,1)/(a+dqK1(3,i));
	}
	// cerr << "step1 good" << endl;

	possNewError.block(0,2*__N__,3,1) << 0,0,0;
	possNewError.bottomRows(3) <<  chi.bottomRows(3);
	return possNewError;
}