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
#include <uav_abstraction_layer/ual.h>
#include <uav_abstraction_layer/backend.h>
#include <ual_backend_mavros_fw/ual_backend_mavros_fw.h>
#include <ros/ros.h>

int main(int _argc, char** _argv) {

    ros::init(_argc, _argv, "ual_server");

    grvc::ual::UAL ual(new grvc::ual::BackendMavrosFW());

    int uav_id;
    ros::param::param<int>("~uav_id", uav_id, 1);

    while (!ual.isReady() && ros::ok()) {
        ROS_WARN("UAL %d not ready!", uav_id);
        sleep(1);
    }
    ROS_INFO("UAL %d ready!", uav_id);
    while (ros::ok()) { sleep(1); }

    return 0;
}