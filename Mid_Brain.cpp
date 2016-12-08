/*!
 Created: Dec. 4 2016
 Mid-brain functions using lidar to output a velocity to the arbiter
*/

#include <string>
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int8MultiArray.h"


std_msgs::Int8MultiArray cmd_array;

std::vector<float> average_ranges;

sensor_msgs::LaserScan scan;
sensor_msgs::LaserScan filtered_scan;


ros::NodeHandle n;

// ros::Publisher pub_arb;
ros::Publisher pub_flag = n.advertise<std_msgs::Int8>("obst/avoid",1000);
ros::Publisher pub_vel =n.advertise<std_msgs::Int8MultiArray>("obst/cmd_vel", 1000);
// ros::Publisher pub_ang =n.advertise<std_msgs::Int8MultiArray>("obst/cmd_dir", 1000);
ros::Publisher pub_filtered_scan =n.advertise<sensor_msgs::LaserScan>("obst/filtered_scan", 1000);
// ros::Publisher pub_arb =n.advertise<std_msgs::Int8MultiArray>("obst/arb", 1000);


// ros::Publisher pub_ang;

std_msgs::Int8 flag;


int rolling_length = 5;

int backward[22] = {0, 0, 0, 0, 2, 0, 0, 0, 0, 0, 0,
										0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int stop[22] =     {0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0,
										0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int forward[22] =  {0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 0,
										0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int right[22] =   {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
										0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0};
int straight[22] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
										0, 0, 0, 0, 0, 2, 0, 0, 0, 0, 0};
int left[22] =     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
										0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0};


void controlSpeed(const sensor_msgs::LaserScan lidar_scan)
{
	// DEBUG
	ROS_INFO("Received Scan");



	// Assign LIDAR scan to global
	scan = lidar_scan;

	long number_of_ranges = lidar_scan.ranges.size();
	float forward_distance = 100;

	// Remove junk values from scan data (0.0 is out of range or no read)
	for(int i=0; i < sizeof(scan.ranges) / sizeof(scan.ranges[0]); i++)
	{
		if(scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max)
		{
			scan.ranges[i] = 0.0;
		}
	}

	// Calculate output array using some portion of scan
	for (long i = number_of_ranges/3; i<2*number_of_ranges/3;i++)
	{
			if (forward_distance > scan.ranges[i] && scan.ranges[i] != 0)
					forward_distance = scan.ranges[i];
	}


	ROS_INFO("Forward distance: %lf", forward_distance);
	if (forward_distance < .3)
	{
		// Move backward
		flag.data=1; //avoiding obstacles
		ROS_INFO("Backward");
		cmd_array.data.assign(backward, backward+22);
	}
	else if (forward_distance < .6)
	{
		// Stop
		flag.data=1; //avoiding obstacles
		ROS_INFO("Stop");
		cmd_array.data.assign(stop, stop+22);
	}
	else
	{
		// Move forward
		flag.data=0; //not avoiding obstacles
		ROS_INFO("Forward");
		cmd_array.data.assign(forward, forward+22);
		//ROS_INFO_STREAM(cmd_array);
	}


	pub_vel.publish(cmd_array);
	pub_flag.publish(flag);
	cmd_array.data.clear();

	// DEBUG
	ROS_INFO("Publishing Output");

}

void getLIDAR(const sensor_msgs::LaserScan lidar_scan)
{
	//ROS_INFO("Received Scan");

	scan.ranges = lidar_scan.ranges;
	filtered_scan.ranges = lidar_scan.ranges;
	filtered_scan.header.frame_id = lidar_scan.header.frame_id;
	filtered_scan.angle_min = lidar_scan.angle_min;
	filtered_scan.angle_max = lidar_scan.angle_max;
	filtered_scan.angle_increment = lidar_scan.angle_increment;
	filtered_scan.range_max = lidar_scan.range_max;
	filtered_scan.range_min = lidar_scan.range_min;
	std::vector<int> indices;

	long number_of_ranges = lidar_scan.ranges.size();

	float average_range = 0.0;

	float rolling_average_range = 0.0;

	// Remove junk values from scan data (0.0 is out of range or no read)
	for(int i=0; i < number_of_ranges; i++)
	{
		if(filtered_scan.ranges[i] < filtered_scan.range_min || filtered_scan.ranges[i] > filtered_scan.range_max)
				filtered_scan.ranges[i] = 0.0;
	}

		for(int i = 0; i < number_of_ranges/6; i++)
		{
				filtered_scan.ranges[i] = 0;
		}

		int distances_counted = 0;

		for(float i = number_of_ranges/6+42; i < number_of_ranges / 4+42; i++)
		{
				if(!isnan(filtered_scan.ranges[i])) {
						average_range += filtered_scan.ranges[i];
						distances_counted++;
				}
		}

		for(float i = number_of_ranges/4; i < number_of_ranges; i++)
		{
				filtered_scan.ranges[i] = 0;
		}

		// Calculate average range
		average_range /= distances_counted;

		// Remove oldest range if vector is of certain size
		if (average_ranges.size() == rolling_length)
		{
			average_ranges.erase(average_ranges.begin(), average_ranges.begin()+1);
		}

		// Add newest range to vector
		average_ranges.push_back(average_range);

		// Calculate running average of ranges
		//rolling_average_range = 1.0 * std::accumulate(average_ranges.begin(), \
			average_ranges.end(), 0.0) / average_ranges.size();
		float sum=0;

		//adding up the array
		for ( int i=0; i<average_ranges.size(); i++ ) sum += average_ranges[i];
		rolling_average_range=sum/average_ranges.size(); //final average



		ROS_INFO("average_range: %f", average_range);
		ROS_INFO("rolling_average_range: %f", rolling_average_range);

		if(rolling_average_range < 1.2)
				cmd_array.data.assign(right, right+22);
		else if(rolling_average_range > 1.65)
				cmd_array.data.assign(straight, straight+22);
		else
				cmd_array.data.assign(left, left+22);

	pub_filtered_scan.publish(filtered_scan);

	pub_vel.publish(cmd_array);
	cmd_array.data.clear();

}

ros::Subscriber sub_imu = n.subscribe("scan", 1000, controlSpeed);
ros::Subscriber sub_lidar = n.subscribe("scan",1000,getLIDAR);

//I probably messed something up in implementation here. If someone could check me that'd be great
int main(int argc, char **argv)
{
	flag.data=0;
	cmd_array.data.assign(&stop[0], &stop[0]+22);

	ros::init(argc, argv, "midbrain");




	ros::spin();

	return 0;
}