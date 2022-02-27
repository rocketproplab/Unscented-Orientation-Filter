#include <math.h>
#include "main.hpp"

const int a = 1;                            // fine tuning constant
const int lambda = 1;                       // fine tuning constant
const int f = 2*(a+1);                      // fine tuning constant
const int n = 6;                            // half the number of sigma points

// initial attitude quaternion estimate
const double initial_attitudeQuat [4] = {0,0,0,1};

// initial gyroscope bias estimate in rad/s
const double initial_gyroBias [3] = {0,0,0};

// initial attitude error and bias error components of the covariance in rad
const double initial_cov_attitude_error = 7.615435e-5;
const double initial_cov_bias_error = 9.401772e-13;

const double sigma_bias = 3.1623e-10;   // estimated gyro bias standard dev
const double sigma_noise = 0.21623e-6;  // estimated gyro noise standard dev
const double sigma_mag = 50e-9;         // estimated mag noise standard dev
const double gyroDt = 10;               // time in s between gyro measurements

const double noiseCov [6][6] = {{(gyroDt/2)*(sigma_noise * sigma_noise -
                                sigma_bias * sigma_bias * gyroDt * gyroDt / 6),
                                0, 0, 0, 0, 0}, {0, (gyroDt/2)*(sigma_noise *
                                sigma_noise - sigma_bias * sigma_bias * gyroDt
                                * gyroDt / 6), 0, 0, 0}, {0, 0, (gyroDt/2)*
                                (sigma_noise * sigma_noise - sigma_bias *
                                sigma_bias * gyroDt * gyroDt / 6), 0, 0, 0,},
                                {0, 0, 0, sigma_bias*sigma_bias, 0, 0}, {0, 0,
                                0, 0, sigma_bias*sigma_bias, 0}, {0, 0, 0, 0, 0,
                                sigma_bias*sigma_bias}};
