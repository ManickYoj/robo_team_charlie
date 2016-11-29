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
        ros::Publisher testing;
		ros::Publisher logging;
		ros::Publisher output;
		ros::Subscriber gpsSub;
		ros::Subscriber compassSub;
		ros::Subscriber waypointSub;
		void recalculateHeading();
		void chatter(std::string s);
        void storeGPS();

	public:
		DirectionFinder() {
			logging = n.advertise<std_msgs::String>("/chatter", 1000);
			output = n.advertise<std_msgs::Int16MultiArray>("/wpt/cmd_vel", 1000);


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
    void storeGPS();
    /*
        TODO: Use stored data to calculate a heading from
        the current position to the waypoint.
    */

    // TODO: Publish the resultant heading.
    // output.publish(<message>);

    return;
}


void DirectionFinder::storeGPS() {
    // Return early if insufficient information available
    if (waypoint == NULL) return;
    if (heading == NULL) return;
    if (position == NULL) return;

    std::vector <float> gps_storedx;
    std::vector <float> gps_storedy;
    std::int num_points;
    gps_storedx.push_back(position[0]);
    gps_storedy.push_back(position[1]);
    num_points = gps_storedx.size();



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
		double correctedHeading = uncorrectedHeading + DECLINATION;
		if (correctedHeading > M_PI) correctedHeading -= M_PI*2;
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
