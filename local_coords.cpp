#include <vector>
#include <iostream>
#include <tf/transform_datatypes.h>

#include <fstream>

tf::Point local_coords(double current_long, double current_lat) {
/*
    std::
    std::ifstream wpt_file("waypoints.json", std::ifstream::binary);
    wpt_file >> wpt;

    std::cout << wpt; //This will print the entire json object.

//The following lines will let you access the indexed objects.
    std::cout << wpt["s25"]; //Prints the value for s25

    std::cout << wpt["s25"]["latitude"]; //Prints the value corresponding to "latitude" in the json for "s25"
*/
    double latitudeE = 42.29335375;
    double longitudeE = -71.26358725;

    double lat_diff = latitudeE - current_lat;
    double long_diff = longitudeE - current_long;
    std::cout << lat_diff << "\n";
    std::cout << long_diff << "\n";

    double y_meters = lat_diff * 111078.95354277734;
    double x_meters = long_diff * 82469.1107701757;

    return tf::Point(x_meters, y_meters, 0);
}


int main(int argc, char* argv[])
{
    double current_lat = 42.29363775;
    double current_long = -71.2638475;
    tf::Point local_xyz= local_coords(current_long, current_lat);
    std::cout << local_xyz.x();
    std::cout << local_xyz.y();
    return 0;
}
