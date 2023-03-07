#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "costmap_2d_test");
    ros::NodeHandle nh;
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener tfl(buffer);
    ROS_INFO("111");
    costmap_2d::Costmap2DROS* mLocalCostmapROS = new costmap_2d::Costmap2DROS("local_costmap", buffer);
    ROS_INFO("222");
    return 0;
}
