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
#ifndef UAV_ABSTRACTION_LAYER_POSEPID_H
#define UAV_ABSTRACTION_LAYER_POSEPID_H

#include <uav_abstraction_layer/PID.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace grvc { namespace ual {

class PosePID {
public:
    PosePID(PIDParams _x_params, PIDParams _y_params, 
        PIDParams _z_params, PIDParams _yaw_params) :
        pid_x_(_x_params), pid_y_(_y_params), pid_z_(_z_params), pid_yaw_(_yaw_params) {}

    bool enableRosInterface(std::string _tag) {
        pid_x_.enableRosInterface(_tag + "/x");
        pid_y_.enableRosInterface(_tag + "/y");
        pid_z_.enableRosInterface(_tag + "/z");
        pid_yaw_.enableRosInterface(_tag + "/yaw");
    }

    void reference(geometry_msgs::PoseStamped _ref_pose) {
        pid_x_.reference(_ref_pose.pose.position.x);
        pid_y_.reference(_ref_pose.pose.position.y);
        pid_z_.reference(_ref_pose.pose.position.z);
        pid_yaw_.reference(quaternion2Yaw(_ref_pose.pose.orientation));
    }

    geometry_msgs::TwistStamped update(geometry_msgs::PoseStamped _cur_pose) {
        geometry_msgs::TwistStamped velocity;
        auto time_inc = _cur_pose.header.stamp - last_pose_.header.stamp;

        velocity.twist.linear.x = pid_x_.update(_cur_pose.pose.position.x, time_inc.toSec());
        velocity.twist.linear.y = pid_y_.update(_cur_pose.pose.position.y, time_inc.toSec());
        velocity.twist.linear.z = pid_z_.update(_cur_pose.pose.position.z, time_inc.toSec());
        velocity.twist.angular.x = 0;
        velocity.twist.angular.y = 0;
        velocity.twist.angular.z = pid_yaw_.update(quaternion2Yaw(_cur_pose.pose.orientation), time_inc.toSec());
        velocity.header.frame_id = _cur_pose.header.frame_id;
        velocity.header.stamp = ros::Time::now();
        
        last_pose_ = _cur_pose;

        return velocity;
    }

    void reset() {
        pid_x_.reset();
        pid_y_.reset();
        pid_z_.reset();
        pid_yaw_.reset();
    }

private:
    float quaternion2Yaw(geometry_msgs::Quaternion q) {
        return atan2( 2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z) );
    }

    PID pid_x_;
    PID pid_y_;
    PID pid_z_;
    PID pid_yaw_;

    geometry_msgs::PoseStamped last_pose_;
};

}}

#endif // UAV_ABSTRACTION_LAYER_POSEPID_H