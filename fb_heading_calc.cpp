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
#include "std_msgs/Int16MultiArray.h"



// -- Constant Declarations
const double DECLINATION = 0.2549926;

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
			output = n.advertise<std_msgs::Int16MultiArray>("/wpt/cmd_vel", 1000);


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

	// TODO: Publish the resultant heading.
	// output.publish(<message>);

	return;
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
void DirectionFinder::updateHeading (const geometry_msgs::Vector3Stamped &heading)
{
		// x seems to 0 out when pointed west, which accords with when the IMU's front
		// is pointed to magnetic north
		double uncorrectedHeading = heading.vector.x;

		// Correct declination (difference between true north and magnetic north)
		// and be sure to limit values from -PI to PI
		double correctedHeading = uncorrectedHeading + DECLINATION;
		if (correctedHeading > M_PI) correctedHeading -= M_PI*2;
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
