// #include <unistd.h>
#include "simulation.hpp"
#include <iostream>
#define PI 3.1415926535d
#define ATT_ERR PI * PI / (360 * 360)
#define BIAS_ERR PI * PI / (3.24E6 * 3.24E6)


int main(int argc, char *argv[])
{
	Usque::Simulation sim;
	sim.run();
	sim.printOutput();
	return 0;
}
