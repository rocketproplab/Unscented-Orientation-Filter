# Unscented-Orientation-Filter

A MATLAB implementation of Crassidis and Markley's Unscented Quaternion Estimator (USQUE) for spacecraft orientation, with sensor measurment simulations and performance comparisons to naive quaternion integration. This unscented filter reads 3-axis gyroscope and magnetometer data, and estimates and removes noise and bias in the gyroscope data by comparing the magnetometer measurements to the expected measurements from the world magnetic model. This filter uses quaternions and modified Rodriguez parameters to keep track of the orientation of the spacecraft. You can find the research paper this is based on [here!](https://doi.org/10.2514/2.5102)


## Using this code (Matlab)

Download this code and open testFilter.m and runFilter.m in the MATLAB IDE. Try running testFilter.m


### Simulating USQUE performance for different flights
To configure the simulation you would like to run, open `testFilter.m`, `runFilter.m`, `idealPath.m`, `readGyro.m`, and `readMag.m`. In `testFilter.m`, set the Kalman filter's knowledge of the initial orientation quaternion, initial covariance, and initial bias. Set the true initial orientation quaternion in `idealPath.m`, and the true initial bias in `readGyro.m` (make these different from the values in `testFilter.m` to see how USQUE corrects errors). In `runFilter.m`, Under "Simulation Parameters", set the time it takes for the spacecraft to complete a rotation, the vector the spacecraft turns around, the runtime of the simulation, the sensor sampling interval, and the filter's knowledge of gyroscope bias standard deviation, gyroscope noise standard deviation, and magnetometer noise standard deviation. Set the true gyroscope noise and bias standard deviations in `readGyro.m`, and set the true magnetometer noise standard deviation in `readMag.m`.
    
### Fine tuning USQUE
Try simulating your flight with different values of a and lambda listed under "Constants and Fine-Tuning" in runFilter.m. Crassidis and Markley recommend a=1 and lambda=1.

### Implementing on a spacecraft
Implementing this filter on a spacecraft requires estimating gyroscope and magnetometer noise parameters. Look at Eq. (25) and (26). We are modeling the gyro measurement as the true angular rate, plus a bias value, plus a noise value. Both the noise and changes in the bias are their own Gaussian white noise processes. We should check the gyro documentation but it is unlikely to help.

If we can spin the gyro at an exact rate known to us (keeping it perfectly still might work) then the average deviation of the measurement from the actual rate for the first time steps will roughly equal the gyro bias. If we repeat this many times, we should get a good estimate for the typical initial bias. Afterwards, we can compare the individual measurements to this to find the noise standard devition. I am unsure how to find the bias change standard deviation.

The paper says "Note that the actual magnetic field errors have systematic components, but these are not relevant to the present filter comparisons."


# Building For C++
This project relies on `Eigen 3.4.0`.

To build using CMake, use the following commands.
```bash
cmake -S . -B build
cmake --build build
```
To run unit tests, use the command after building the CMake build directory:
```bash
cd build/tests && ctest
```

## Simulation
The simulation class is found in `src/simulation`. To use it: 
1. Construct a `Usque::Simulation` instance. Set its parameters, either though the explicit constructor or using the builder pattern (TODO).
2. Call `instance.run()`. Note that you should not modify parameters during this stage, and this method should only be called once.
3. Call `instance.output()`. This outputs the simulation results into a CSV file.

### Example
This code has not been tested yet. (TODO: Implement this usage pattern)

```cpp
#include "simulation.hpp"

int main() {
	//Create a simulation with default params
	Usque::Simulation sim;
	//Run.
	sim.run();
	//Output results to path/to/result as a CSV file.
	sim.output("path/to/result");
}

```

# License
Eigen 3.4 uses the [MPL 2.0 License](https://www.mozilla.org/en-US/MPL/2.0/).

GoogleTest uses the [BSD 3-Clause "New" or "Revised" License](https://github.com/google/googletest/blob/main/LICENSE).

# Roadmap
Currently in the process of implementing USQUE in C++.

1. Implementation
	1. Simulate inputs: read gyro, WMM, etc.
	2. Extract outputs

2. Testing
	1. Unit tests for all functions.
	2. Integration Tests