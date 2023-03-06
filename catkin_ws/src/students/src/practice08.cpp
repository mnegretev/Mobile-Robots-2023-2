/*
 * MOBILE ROBOTS - UNAM, FI, 2023-2
 * PRACTICE 8 - SIMULTANEOUS LOCALIZATION AND MAPPING
 *
 * Instructions:
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"
#include "random_numbers/random_numbers.h"
#include "occupancy_grid_utils/ray_tracer.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_broadcaster.h"

#define NOMBRE "APELLIDO_PATERNO_APELLIDO_MATERNO"

int main(int argc, char** argv)
{
    std::cout << "PRACTICE 08 - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practice08");
    ros::NodeHandle n("~");
    ros::Rate loop(20);
    
    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
