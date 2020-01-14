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

#include <ual_backend_ue/ual_backend_ue.h>

namespace grvc { namespace ual {

BackendUE::BackendUE() : Backend()
{
    ned2enu_ =  Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitZ())*
                Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX());
    // Check connection with simulation
    airsim_client_.confirmConnection();

    // Enable external control and arm the UAV
    airsim_client_.enableApiControl(true);
    airsim_client_.armDisarm(true);

    state_ = uav_abstraction_layer::State::LANDED_ARMED;

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
    Eigen::Vector3f position = {pose_ue.position[0],
                                pose_ue.position[1],
                                pose_ue.position[2]};
    Eigen::Quaternionf ori( pose_ue.orientation.w(),
                            pose_ue.orientation.x(),
                            pose_ue.orientation.y(),
                            pose_ue.orientation.z());

    Eigen::Vector3f newPos = ned2enu_*position;
    Eigen::Quaternionf newOri(ned2enu_*ori.matrix());

    Pose pose_ual;
    pose_ual.header.stamp = ros::Time::now();
    pose_ual.header.frame_id = "";
    pose_ual.pose.position.x = newPos[0];
    pose_ual.pose.position.y = newPos[1];
    pose_ual.pose.position.z = newPos[2];
    pose_ual.pose.orientation.x = newOri.x();
    pose_ual.pose.orientation.y = newOri.y();
    pose_ual.pose.orientation.z = newOri.z();
    pose_ual.pose.orientation.w = newOri.w();
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
    
    Eigen::Vector3f position = {pose_ue.position[0],
                                pose_ue.position[1],
                                pose_ue.position[2]};
    Eigen::Quaternionf ori( pose_ue.orientation.w(),
                            pose_ue.orientation.x(),
                            pose_ue.orientation.y(),
                            pose_ue.orientation.z());

    Eigen::Vector3f newPos = ned2enu_*position;
    Eigen::Quaternionf newOri(ned2enu_*ori.matrix());

    Transform out;
    out.header.stamp = ros::Time::now();
    out.header.frame_id = "";
    out.child_frame_id = "";
    out.transform.translation.x = newPos[0];
    out.transform.translation.y = newPos[1];
    out.transform.translation.z = newPos[2];
    out.transform.rotation.x = newOri.x();
    out.transform.rotation.y = newOri.y();
    out.transform.rotation.z = newOri.z();
    out.transform.rotation.w = newOri.w();
    return out;
}

void BackendUE::setPose(const geometry_msgs::PoseStamped& _pose) {}

void BackendUE::goToWaypoint(const Waypoint& _wp) {
    // Get heading
    Eigen::Quaternionf q(_wp.pose.orientation.w, _wp.pose.orientation.x, _wp.pose.orientation.y, _wp.pose.orientation.z);
    auto angles = q.toRotationMatrix().eulerAngles(0, 1, 2);
    double yaw = angles[2] * 180.0f/ M_PI;
    // Move UAV
    airsim_client_.moveToPositionAsync( _wp.pose.position.x, 
                                        _wp.pose.position.y,
                                        _wp.pose.position.z, 
                                        5   ,  // Speed m/s 
                                        Utils::max<float>(),
                                        DrivetrainType::MaxDegreeOfFreedom, 
                                        { false, yaw }
                                        );
}

void BackendUE::goToWaypointGeo(const WaypointGeo& _wp) {}

void BackendUE::takeOff(double _height) {
    /// 666 TODO: Check altitude
    airsim_client_.takeoffAsync()->waitOnLastTask();

    state_ = uav_abstraction_layer::State::FLYING_AUTO;
}

void BackendUE::land() {
    airsim_client_.landAsync()->waitOnLastTask();
    
    state_ = uav_abstraction_layer::State::LANDED_ARMED;
}

void BackendUE::setVelocity(const Velocity& _vel) {
    Eigen::Vector3f rotvel = {  _vel.twist.linear.x,
                                _vel.twist.linear.y,
                                _vel.twist.linear.z };
    rotvel = ned2enu_*rotvel;

    airsim_client_.moveByVelocityAsync(    rotvel[0], 
                                           rotvel[1], 
                                           rotvel[2], 
                                            Utils::max<float>(),
                                            DrivetrainType::MaxDegreeOfFreedom, 
                                            {true, -_vel.twist.angular.z * 180.0f/ M_PI});
}

void BackendUE::recoverFromManual() {}

void BackendUE::setHome(bool set_z) {}

}} // namespace grvc::ual


#endif  // UAL_UE_COMPATIBILITY