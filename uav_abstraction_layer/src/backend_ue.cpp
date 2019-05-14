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


#ifdef UAL_UE_COMPATIBILITY

//#include <string>
//#include <chrono>
//#include <iostream>

#include <common/common_utils/StrictMode.hpp>
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include <rpc/rpc_error.h>
STRICT_MODE_ON

#include <common/common_utils/FileSystem.hpp>

#include <uav_abstraction_layer/backend_ue.h>

namespace grvc { namespace ual {

BackendUE::BackendUE() : Backend()
{
    // Check connection with simulation
    airsim_client_.confirmConnection();

    // Enable external control and arm the UAV
    airsim_client_.enableApiControl(true);
    airsim_client_.armDisarm(true);

}

BackendUE::~BackendUE() {

}

bool BackendUE::isReady() const {
    //auto state = airsim_client_.getConnectionState();
    //return  state == msr::airlib::RpcLibClientBase::ConnectionState::Connected;
    return true;
}

grvc::ual::Pose BackendUE::pose() {
    auto pose_ue  = airsim_client_.simGetVehiclePose();
    Pose pose_ual;
    pose_ual.header.stamp = ros::Time::now();
    pose_ual.header.frame_id = "";
    pose_ual.pose.position.x = pose_ue.position[0];
    pose_ual.pose.position.y = pose_ue.position[1];
    pose_ual.pose.position.z = pose_ue.position[2];
    pose_ual.pose.orientation.x = pose_ue.orientation.x();
    pose_ual.pose.orientation.y = pose_ue.orientation.y();
    pose_ual.pose.orientation.z = pose_ue.orientation.z();
    pose_ual.pose.orientation.w = pose_ue.orientation.w();
    return pose_ual;
}

grvc::ual::Velocity BackendUE::velocity() const {
    return Velocity();
}

grvc::ual::Odometry BackendUE::odometry() const {
    return Odometry();
}

grvc::ual::Transform BackendUE::transform() const {
    auto pose_ue  = airsim_client_.simGetVehiclePose();
    Transform out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = "";
    out.child_frame_id = "";
    out.transform.translation.x = pose_ue.position[0];
    out.transform.translation.y = pose_ue.position[1];
    out.transform.translation.z = pose_ue.position[2];
    out.transform.rotation.x = pose_ue.orientation.x();
    out.transform.rotation.y = pose_ue.orientation.y();
    out.transform.rotation.z = pose_ue.orientation.z();
    out.transform.rotation.w = pose_ue.orientation.w();
    return out;
}

void BackendUE::setPose(const geometry_msgs::PoseStamped& _pose) {}

void BackendUE::goToWaypoint(const Waypoint& _wp) {}

void BackendUE::goToWaypointGeo(const WaypointGeo& _wp) {}

void BackendUE::takeOff(double _height) {
    /// 666 TODO: Check altitude
    airsim_client_.takeoffAsync()->waitOnLastTask();
}

void BackendUE::land() {
    airsim_client_.landAsync()->waitOnLastTask();
}

void BackendUE::setVelocity(const Velocity& _vel) {}

void BackendUE::recoverFromManual() {}

void BackendUE::setHome(bool set_z) {}

}} // namespace grvc::ual


#endif  // UAL_UE_COMPATIBILITY