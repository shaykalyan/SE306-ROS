#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include "math.h"
#include <vector>
#include "elderly_care_simulation/FindPath.h"

#include <unistd.h> // sleep

#include "gtest/gtest.h"

using namespace elderly_care_simulation;

/**
 * The fixture for testing the Scheduler.
 * 
 * Most of these methods could be removed, but they have been left as
 * an example.
 */
class PathServerTest : public ::testing::Test {
	protected:
		// You can remove any or all of the following functions if its body
		// is empty.

		PathServerTest() {
			// You can put any initialisation here.
		}

		virtual ~PathServerTest() {
			// You can do clean-up work that doesn't throw exceptions here.
		}

		// If the constructor and destructor are not enough for setting up
		// and cleaning up each test, you can define the following methods:

		virtual void SetUp() {
			// Code here will be called immediately after the constructor (right
			// before each test).
		}

		virtual void TearDown() {
			// Code here will be called immediately after each test (right
			// before the destructor).
		}
};

ros::ServiceClient pathFinderService;

elderly_care_simulation::FindPath findPath(float x_from, float y_from, float x_to, float y_to) {
	geometry_msgs::Point to;
	geometry_msgs::Point from;

	to.x = x_to;
	to.y = y_to;
	from.x = x_from;
	from.y = y_from;

	elderly_care_simulation::FindPath srv;
	srv.request.from_point = to;
	srv.request.from_point = from;
	return srv;
}

TEST_F(PathServerTest, requestPath) {
	
	elderly_care_simulation::FindPath srv = findPath(0.0f, 0.0f, 0.0f, 0.0f);
	ASSERT_TRUE(pathFinderService.call(srv));
	ASSERT_TRUE(srv.response.path.size() == 0);

	srv = findPath(15.0f, 15.0f, 0.0f, 0.0f);
	ASSERT_FALSE(pathFinderService.call(srv));

	srv = findPath(16.0f, 16.0f, 0.0f, 0.0f);
	ASSERT_FALSE(pathFinderService.call(srv));

	srv = findPath(12.0f, 12.0f, -12.0f, 12.0f);
	ASSERT_TRUE(pathFinderService.call(srv));
	ASSERT_FLOAT_EQ(12.0f, srv.response.path[0].x);
	ASSERT_FLOAT_EQ(11.0f, srv.response.path[0].y);

	srv = findPath(12.0f, 12.0f, -160.0f, 0.0f);
	ASSERT_TRUE(pathFinderService.call(srv));
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "TestPathServer");
	ros::NodeHandle nodeHandle;
	sleep(1);
	pathFinderService = nodeHandle.serviceClient<elderly_care_simulation::FindPath>("find_path");

	// Run tests to see if we received messages as expected
	testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}