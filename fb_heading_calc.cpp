#include <vector>
#include <iostream>
#include <stddef.h> //Defines NULL

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/NavSatFix.h"

class DirectionFinder {
	private:
		geometry_msgs::Vector3 *compassOrientation;
		sensor_msgs::NavSatFix *gpsPosition;
		sensor_msgs::NavSatFix *waypoint;
		ros::NodeHandle *n;
		ros::Publisher *output;
		void recalculateHeading();

	public:
		DirectionFinder(ros::NodeHandle &n);
		void updateGPS(const sensor_msgs::NavSatFix &gpsPosition);
		void updateWaypoint(const sensor_msgs::NavSatFix &gpsPosition);
		void updateCompass(const geometry_msgs::Vector3Stamped &compassOrientation);
};

/**
	Construct the DirectionFinder class.waypoint

	@param n A NodeHandle to use as the base for publications and subscriptions
*/
DirectionFinder::DirectionFinder(ros::NodeHandle &n) {
	this->n = &n;
	ros::Publisher output = n.advertise<geometry_msgs::Vector3>("/desired_heading", 1000);
	this->output = &output;

	this->compassOrientation = NULL;
	this->gpsPosition = NULL;
	this->waypoint = NULL;
}

/**
	To be invoked when any of the DirectionFinder data fields
	are updated, this function uses the stored data to calculate
	a new desired heading for the robot and publishes it to the
	topic targeted by DirectionFinder->output.
*/
void DirectionFinder::recalculateHeading() {
	// Return early if insufficient information available
	if (waypoint == NULL) return;
	if (compassOrientation == NULL) return;
	if (gpsPosition == NULL) return;

	/*
		TODO: Use stored data to calculate a heading from
		the current gpsPosition to the waypoint.
	*/

	// TODO: Publish the resultant heading.
	// output.publish(<message>);

	return;
}

/**
	Updates the gpsPosition field and triggers a heading recalculation.

	@param gpsPosition
*/
void DirectionFinder::updateGPS (const sensor_msgs::NavSatFix &gpsPosition)
{
    this->gpsPosition = (sensor_msgs::NavSatFix*) &gpsPosition;
    this->recalculateHeading();
}

/**
	Updates the gpsPosition field and triggers a heading recalculation.

	@param waypoint
*/
void DirectionFinder::updateWaypoint (const sensor_msgs::NavSatFix &waypoint)
{
    this->waypoint = (sensor_msgs::NavSatFix*) &waypoint;
    this->recalculateHeading();
}

/**
	Updates the compassOrientation field and triggers a heading recalculation.

	@param compassOrientation
*/
void DirectionFinder::updateCompass (const geometry_msgs::Vector3Stamped &compassOrientation)
{
    this->compassOrientation = (geometry_msgs::Vector3*) &(compassOrientation.vector);
    this->recalculateHeading();
}



int main(int argc, char* argv[])
{
	// Initialize DirectionFinder
	ros::init(argc, argv, "gps_ros");
    ros::NodeHandle n;
    DirectionFinder d(n);

    // Set up subscriptions to required data sources:
    // The DirectionFinder will recalculate and publish headings
    // once all of the topics on which it is dependent have been
    // published to.
    ros::Subscriber subGPS = n.subscribe("fix", 1000, &DirectionFinder::updateGPS, &d);
    ros::Subscriber subCompass = n.subscribe("imu/mag", 1000, &DirectionFinder::updateCompass, &d);
    ros::Subscriber subWaypoint = n.subscribe("/waypoint", 1000, &DirectionFinder::updateWaypoint, &d);

    ros::spin();
}
