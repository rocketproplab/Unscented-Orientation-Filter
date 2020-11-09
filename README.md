# Unscented-Orientation-Filter

	A MATLAB implementation of Crassidis and Markley's Unscented Quaternion Estimator (USQUE) for spacecraft orientation, with sensor measurment simulations and performance comparisons to naive quaternion integration. This unscented filter reads 3-axis gyroscope and magnetometer data, and estimates and removes noise and bias in the gyroscope data by comparing the magnetometer measurements to the expected measurements from the world magnetic model. This filter uses quaternions and modified Rodriguez parameters to keep track of the orientation of the spacecraft. You can find the research paper this is based on [here!](https://doi.org/10.2514/2.5102)


## Using this code

    Download this code and open testFilter.m and runFilter.m in the MATLAB IDE. Try running testFilter.m


### Simulating USQUE performane for different flights
	To configure the simulation you would like to run, open `testFilter.m`, `runFilter.m`, `idealPath.m`, `readGyro.m`, and `readMag.m`. In `testFilter.m`, set the Kalman filter's knowledge of the initial orientation quaternion, initial covariance, and initial bias. Set the true initial orientation quaternion in `idealPath.m`, and the true initial bias in `readGyro.m` (make these different from the values in `testFilter.m` to see how USQUE corrects errors). In `runFilter.m`, Under "Simulation Parameters", set the time it takes for the spacecraft to complete a rotation, the vector the spacecraft turns around, the runtime of the simulation, the sensor sampling interval, and the filter's knowledge of gyroscope bias standard deviation, gyroscope noise standard deviation, and magnetometer noise standard deviation. Set the true gyroscope noise and bias standard deviations in `readGyro.m`, and set the true magnetometer noise standard deviation in `readMag.m`.
    
### Fine tuning USQUE
	Try simulating your flight with different values of a and lambda listed under "Constants and Fine-Tuning" in runFilter.m. Crassidis and Markley recommend a=1 and lambda=1.

### Implementing on a spacecraft
    Implementing this filter on a spacecraft requires estimating gyroscope and magnetometer noise parameters. Look at Eq. (25) and (26). We are modeling the gyro measurement as the true angular rate, plus a bias value, plus a noise value. Both the noise and changes in the bias are their own Gaussian white noise processes. We should check the gyro documentation but it is unlikely to help.

    If we can spin the gyro at an exact rate known to us (keeping it perfectly still might work) then the average deviation of the measurement from the actual rate for the first time steps will roughly equal the gyro bias. If we repeat this many times, we should get a good estimate for the typical initial bias. Afterwards, we can compare the individual measurements to this to find the noise standard devition. I am unsure how to find the bias change standard deviation.

    The paper says "Note that the actual magnetic field errors have systematic components, but these are not relevant to the present filter comparisons."
