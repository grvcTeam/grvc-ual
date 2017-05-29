//----------------------------------------------------------------------------------------------------------------------
// GRVC UAL
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2016 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>
#include <iostream>

//namespace grvc { namespace ual {

geometry_msgs::TransformStamped getTransform(const char * parent, const char * child, double x, double y, double z, double R, double P, double Y);

int main(int argc, char **argv)
{
    ros::init(argc,argv, "static_tf_publisher");
    //ros::NodeHandle nh;

    int number_of_uavs = 0;

    std::string frame_id;
    std::string parent_frame;
    std::string units;
    std::vector<double> translation;
    std::vector<double> rotation;

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    std::vector<geometry_msgs::TransformStamped> static_transforms_to_publish;

    // Check params
    if (ros::param::has("/map_frame"))
    {
        ros::param::get("/map_frame/frame_id",frame_id);
        ros::param::get("/map_frame/parent_frame",parent_frame);
        ros::param::get("/map_frame/units",units);
        ros::param::get("/map_frame/translation",translation);
        ros::param::get("/map_frame/rotation",rotation);

        static_transforms_to_publish.push_back( getTransform(parent_frame.c_str(),frame_id.c_str(),translation[0],translation[1],translation[2],rotation[0],rotation[1],rotation[2]) );
    }
    if (ros::param::has("/game_frame"))
    {
        ros::param::get("/game_frame/frame_id",frame_id);
        ros::param::get("/game_frame/parent_frame",parent_frame);
        ros::param::get("/game_frame/units",units);
        ros::param::get("/game_frame/translation",translation);
        ros::param::get("/game_frame/rotation",rotation);

        static_transforms_to_publish.push_back( getTransform(parent_frame.c_str(),frame_id.c_str(),translation[0],translation[1],translation[2],rotation[0],rotation[1],rotation[2]) );
    }
    
    static_broadcaster.sendTransform(static_transforms_to_publish);

    ROS_INFO("Spinning until killed publishing map to world");
    ros::spin();
    return 0;
}

geometry_msgs::TransformStamped getTransform(const char * parent, const char * child, double x, double y, double z, double R, double P, double Y)
{
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = parent;
    static_transformStamped.child_frame_id = child;
    static_transformStamped.transform.translation.x = x;
    static_transformStamped.transform.translation.y = y;
    static_transformStamped.transform.translation.z = z;
    tf2::Quaternion quat;
    quat.setRPY(R,P,Y);
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();

    std::cout << "New transform:\n" << static_transformStamped << std::endl;

    return static_transformStamped;
}

//}} // namespace grvc::ual