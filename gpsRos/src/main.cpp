#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>
#include <iostream>

std::vector<int> choose_direction(int gps, int compass){
    // this function takes in the GPS and compass data, and decides what direction (left or right)
    //      the robot should be moving in.

    return direction;
}

ros::Publisher* pub;

void gpsCallback(const std_msgs::Int32::ConstPtr& msg)
{
    // this should put the data how we want it so it can be published
    std::vector<int> direction = choose_direction(msg->data);
    std_msgs::Int16MultiArray direction;
    choose.data = direction;
    pub->publish(choose);

}


int main(int argc, char* argv[]){
    ros::NodeHandle ng;
    ros::NodeHandle nc;
    ros::Subscriber sub = ng.subscribe("gps", 1000, gpsCallback);
    ros::Subscriber sub = nc.subscribe("compass", 1000, compassCallback);

    ros::Publisher wpt_pub = nh.advertise<std_msgs::Int16MultiArray>("/wpt/cmd_vel", 1000);
    pub = &wpt_pub;

    ros::spin();
    //std::cout << num_factors << std::endl;
}
