#define TESTING
namespace UsqueTests {
	class SimWrapper;
}
#include "simulation.hpp"
#include "gtest/gtest.h" //Must include this for tests to work


namespace UsqueTests {
//This class exposes private methods in Usque::Simulation
class SimWrapper {
public:
	Usque::Simulation* sim;

	Eigen::Vector3d getIdealAngV() {
		sim->updateIdealPath();
		return sim->idealAngV;
	}

};

}

class SimTest : public ::testing::Test {
protected:
	Usque::Simulation sim;
	UsqueTests::SimWrapper wrapper;
	//Setup simulation
	void SetUp() override {
		//Setup params if needed
		//sim.setParams([sim]{...});
		wrapper.sim = &sim; 
	}
};

using namespace UsqueTests;



//Test: Ideal Angle Vector
//Uses the SimTest fixture (TEST_F)
TEST_F(SimTest, idealAngV) {
	Eigen::Vector3d ang = wrapper.getIdealAngV();
	// EXPECT_DOUBLE_EQ(ang(0), 0.000310972575622725);
	// EXPECT_DOUBLE_EQ(ang(1), 0.000621945151245450);
	// EXPECT_DOUBLE_EQ(ang(2), 0.000932917726868175);
}