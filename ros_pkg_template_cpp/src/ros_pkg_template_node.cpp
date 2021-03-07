// Copyright 2021-2025
//
// @author: Jose Carlos Garcia
// @email: jcarlos3094@gmail.com
//
#include <ros/ros.h>
#include "ros_pkg_template/ros_pkg_template.hpp"

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    ros::init(argc, argv, "ros_package_template");
    ros::NodeHandle nodeHandle("~");
    
    RosPkgTemplate rosPkgTemplate(nodeHandle);

    ros::spin();
    return 0;
}