#include <vector>
#include <iostream>
#include <stddef.h> //Defines NULL
#include <math.h>
#include <tf/transform_datatypes.h>

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"



// -- Constant Declarations
const double DECLINATION = 0.2549926;
const double LINEAR_MAX = 10;
const double ANGULAR_MAX = 10;

double boundAngle(double angle) {
	if (angle > M_PI) angle -= M_PI*2;
	else if (angle < -M_PI) angle += M_PI*2;
	return angle;
}

double scale(double input, double input_range, double max) {
	return (input/input_range) * max;
}

class DirectionFinder {
	private:
		double* heading;
		tf::Point* position;
		tf::Point* waypoint;
		ros::NodeHandle n;
		ros::Publisher logging;
		ros::Publisher output;
		ros::Subscriber gpsSub;
		ros::Subscriber compassSub;
		ros::Subscriber waypointSub;
		void recalculateHeading();
		void chatter(std::string s);

	public:
		DirectionFinder() {
			logging = n.advertise<std_msgs::String>("/chatter", 1000);
			output = n.advertise<std_msgs::Int8MultiArray>("/wpt/cmd_vel", 1000);


			// Set up subscriptions to required data sources:
		  gpsSub = n.subscribe("/fix", 1000, &DirectionFinder::updateGPS, this);
		  compassSub = n.subscribe("/imu/data", 1000, &DirectionFinder::updateHeading, this);
		  waypointSub = n.subscribe("/waypoint", 1000, &DirectionFinder::updateWaypoint, this);
		}
		void updateGPS(const sensor_msgs::NavSatFix &gpsPosition);
		void updateWaypoint(const sensor_msgs::NavSatFix &waypoint);
		void updateHeading(const sensor_msgs::Imu &heading);
};

/**
	To be invoked when any of the DirectionFinder data fields
	are updated, this function uses the stored data to calculate
	a new desired heading for the robot and publishes it to the
	topic targeted by DirectionFinder->output.
*/
void DirectionFinder::recalculateHeading() {
	// Return early if insufficient information available
	if (waypoint == NULL) return;
	if (heading == NULL) return;
	if (position == NULL) return;

	// Calculate waypoint direction vector
	double deltaX = waypoint->getX()-position->getX();
	double deltaY = waypoint->getY()-position->getY();


	// Angle between heading to waypoint and true North
	// Left is positive, right is negative
	// Note: will error out if deltaY and deltaX are 0, therefore, catch this case
	if (deltaY == 0 && deltaX == 0) return;
	double desiredHeading = boundAngle(atan2(deltaY, deltaX) - M_PI /2);

	// Difference between current heading and heading to waypoint
	// Ideally 0
	double headingChange = boundAngle(desiredHeading - *heading);

	// Test output of angle to waypoint
	std::stringstream ss;
	ss << "Desired heading change: " << headingChange;
	chatter(ss.str());

	// Calculate the angular and linear vel outputs
	double angular_vel = scale(headingChange, M_PI, LINEAR_MAX);
	double linear_vel = LINEAR_MAX;

	// Publish the desired cmd_vel array
	std_msgs::Int8MultiArray cmd_vel;
	cmd_vel.data.push_back(int(linear_vel));
	cmd_vel.data.push_back(int(angular_vel));
	output.publish(cmd_vel);
}

/**
	A simple function to publish a string to the chatter
	topic for debugging purposes.

	@param s The string to publish
*/
void DirectionFinder::chatter(std::string s) {
	std_msgs::String msg;
	msg.data = s;
	ROS_INFO("%s", msg.data.c_str());
	logging.publish(msg);
}

/**
	Updates the position field and triggers a heading recalculation.

	@param gpsPosition
*/
void DirectionFinder::updateGPS (const sensor_msgs::NavSatFix &gpsPosition)
{
		// TODO: Convert GPS position (long-lat-alt) into local frame,
		// with meters as units and the center of the O (center of crossed
		// paths) as the origin

		// (sensor_msgs::NavSatFix*) &gpsPosition;
    // this->gpsPosition = <geometry_msgs::Vector3>

    this->recalculateHeading();
}

/**
	Updates the waypoint field and triggers a heading recalculation.

	@param waypoint
*/
void DirectionFinder::updateWaypoint (const sensor_msgs::NavSatFix &waypoint)
{
		// TODO: Write waypoint node and convert from GPS waypoint to local frame
		// with meters as units and the center of the O (center of crossed paths)
	  // as the origin

    // this->waypoint = (sensor_msgs::NavSatFix*) &waypoint;

    this->recalculateHeading();
}

/**
	Updates the heading field and triggers a heading recalculation.

	@param heading
*/
void DirectionFinder::updateHeading (const sensor_msgs::Imu &heading)
{
		// The fused IMU orientation data comes in as a quaternion,
		// we convert it here to a yaw for ease of use.
		double uncorrectedHeading = tf::getYaw(heading.orientation);

		// Correct declination (difference between true north and magnetic north)
		// and be sure to limit values from -PI to PI
		double correctedHeading = boundAngle(uncorrectedHeading + DECLINATION);
		this->heading = &correctedHeading;

		// Debug output
		std::stringstream ss;
		ss << "Uncorrected: " << uncorrectedHeading << "\nCorrected: " << correctedHeading;
		chatter(ss.str());

    this->recalculateHeading();
}


int main(int argc, char* argv[])
{
	// Initialize DirectionFinder
	ros::init(argc, argv, "heading_calculator");
  DirectionFinder d;
  ros::spin();
  return 0;
}
