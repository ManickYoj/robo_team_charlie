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
const double LINEAR_MAX = 60;
const double ANGULAR_MAX = 100;
const double ORIGIN_LAT = 42.29335375;
const double ORIGIN_LONG = -71.26358725;
const double LAT_TO_M = 111078.95354277734;
const double LONG_TO_M = 82469.1107701757;

double boundAngle(double angle) {
	if (angle > M_PI) angle -= M_PI*2;
	else if (angle < -M_PI) angle += M_PI*2;
	return angle;
}

double scale(double input, double input_range, double max) {
	return (input/input_range) * max;
}

tf::Point toLocalCoords(double longitude, double latitude) {
    double latDiff = ORIGIN_LAT - latitude;
    double longDiff = ORIGIN_LONG - longitude;

    double x_meters = longDiff * LONG_TO_M;
    double y_meters = latDiff * LAT_TO_M ;

    return tf::Point(x_meters, y_meters, 0);
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
		  compassSub = n.subscribe("/imu/mag", 1000, &DirectionFinder::updateHeading, this);
		  waypointSub = n.subscribe("/waypoint", 1000, &DirectionFinder::updateWaypoint, this);
		}
		void updateGPS(const sensor_msgs::NavSatFix &gpsPosition);
		void updateWaypoint(const sensor_msgs::NavSatFix &waypoint);
		void updateHeading(const geometry_msgs::Vector3Stamped &heading);
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
  /*
      TODO: Use stored data to calculate a heading from
      the current position to the waypoint.
  */

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
		// Convert GPS position (long-lat) into local frame

		double latitude = (double) gpsPosition.latitude;
		double longitude= (double) gpsPosition.longitude;
		tf::Point temp = toLocalCoords(latitude, longitude);
		this->position = &temp;

		// Debug Output
		std::stringstream ss;
		ss << "Updated Position: (" << temp.getX() << ", " << temp.getY() << ")";
		chatter(ss.str());

    this->recalculateHeading();
}

/**
	Updates the waypoint field and triggers a heading recalculation.

	@param waypoint
*/
void DirectionFinder::updateWaypoint (const sensor_msgs::NavSatFix &gpsWaypoint)
{
		// Convert GPS waypoint to local frame
		double latitude = (double) gpsWaypoint.latitude;
		double longitude= (double) gpsWaypoint.longitude;
		tf::Point temp = toLocalCoords(latitude, longitude);
		this->waypoint = &temp;

		// Debug Output
		std::stringstream ss;
		ss << "Updated Waypoint: (" << temp.getX() << ", " << temp.getY() << ")";
		chatter(ss.str());

    this->recalculateHeading();
}

/**
	Updates the heading field and triggers a heading recalculation.

	@param heading
*/
void DirectionFinder::updateHeading (const geometry_msgs::Vector3Stamped &heading)
{
		// x seems to 0 out when pointed west, which accords with when the IMU's front
		// is pointed to magnetic north
		double uncorrectedHeading = heading.vector.x;

		// Correct declination (difference between true north and magnetic north)
		// and be sure to limit values from -PI to PI
		double correctedHeading = boundAngle(uncorrectedHeading + DECLINATION);
		this->heading = &correctedHeading;

		// Debug output
		std::stringstream ss;
		ss << "Heading from Magnetic North: " << uncorrectedHeading << "\nHeading from True North: " << heading.vector.y;
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
