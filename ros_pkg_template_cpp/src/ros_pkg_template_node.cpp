// Copyright 2021-2025
//
// @author: Jose Carlos Garcia
// @email: jcarlos3094@gmail.com
//
#include <ros/ros.h>
#include "nodelet/loader.h"

int main(int argc, char * argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    ros::init(argc, argv, "ros_package_template_node");

    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "ros_pkg_template/ros_pkg_template", remap, nargv);
    
    ros::spin();
    return 0;
}