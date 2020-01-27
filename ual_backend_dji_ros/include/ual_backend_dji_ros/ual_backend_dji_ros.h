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
#ifndef UAV_ABSTRACTION_LAYER_BACKEND_DJI_H
#define UAV_ABSTRACTION_LAYER_BACKEND_DJI_H

#include <thread>
// #include <deque>

#include <uav_abstraction_layer/backend.h>
#include <ros/ros.h>
// #include <ros/package.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>

#include <dji_sdk/dji_sdk.h>
// #include <dji_sdk/dji_sdk_node.h>
#include <dji_sdk/Activation.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/MissionWpUpload.h>
#include <dji_sdk/MissionWpSetSpeed.h>
#include <dji_sdk/MissionWpAction.h>


// std_msgs::UInt8 S_FLYING = 2;
typedef double Quaterniond [4];



namespace grvc { namespace ual {

class DjiHistoryBuffer {  // TODO: template? utils?
public:
    void set_size(size_t _size) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_size_ = _size;
        buffer_.clear();
        current_ = 0;
    }

    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.clear();
        current_ = 0;
    }

    void update(double _value) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.size() < buffer_size_) {
            buffer_.push_back(_value);
        } else {
            buffer_[current_] = _value;
            current_ = (current_ + 1) % buffer_size_;
        }
    }

    bool get_stats(double& _min, double& _mean, double& _max) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.size() >= buffer_size_) {
            double min_value = +std::numeric_limits<double>::max();
            double max_value = -std::numeric_limits<double>::max();
            double sum = 0;
            for (int i = 0; i < buffer_.size(); i++) {
                if (buffer_[i] < min_value) { min_value = buffer_[i]; }
                if (buffer_[i] > max_value) { max_value = buffer_[i]; }
                sum += buffer_[i];
            }
            _min = min_value;
            _max = max_value;
            _mean = sum / buffer_.size();
            return true;
        }
        return false;
    }

    bool get_variance(double& _var) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffer_.size() >= buffer_size_) {
            double mean = 0;
            double sum = 0;
            _var = 0;
            for (int i = 0; i < buffer_.size(); i++) {
                sum += buffer_[i];
            }
            mean = sum / buffer_.size();
            for (int i = 0; i < buffer_.size(); i++) {
                _var += (buffer_[i]-mean)*(buffer_[i]-mean);
            }
            return true;
        }
        return false;
    }

protected:
    size_t buffer_size_ = 0;
    unsigned int current_ = 0;
    std::vector<double> buffer_;
    std::mutex mutex_;
};

 
class BackendDjiRos : public Backend {

public:
    BackendDjiRos();
    // ~BackendDjiRos();

    /// Backend is initialized and ready to run tasks?
    bool	         isReady() const override;
    /// Latest pose estimation of the robot
    virtual Pose	 pose() override;
    /// Latest velocity estimation of the robot
    virtual Velocity velocity() const override;
    /// Latest odometry estimation of the robot
    virtual Odometry odometry() const override;
    /// Latest transform estimation of the robot
    virtual Transform transform() const override;
    /// Current reference pose
    virtual Pose referencePose() override;

    /// Set pose
    /// \param _pose target pose
    void    setPose(const geometry_msgs::PoseStamped& _pose) override;

    /// Go to the specified waypoint, following a straight line
    /// \param _wp goal waypoint
    void	goToWaypoint(const Waypoint& _wp) override;

    /// Go to the specified waypoint in geographic coordinates, following a straight line
    /// \param _wp goal waypoint in geographic coordinates
    void	goToWaypointGeo(const WaypointGeo& _wp);

    /// Follow a list of waypoints, one after another
    // void trackPath(const Path& _path) override;
    /// Perform a take off maneuver
    /// \param _height target height that must be reached to consider the take off complete
    void    takeOff(double _height) override;
    /// Land on the current position.
    void	land() override;
    /// Set velocities
    /// \param _vel target velocity in world coordinates
    void    setVelocity(const Velocity& _vel) override;
    /// Recover from manual flight mode
    /// Use it when FLYING uav is switched to manual mode and want to go BACK to auto.
    void    recoverFromManual() override;
    /// Set home position
    void    setHome(bool set_z) override;
private:
    void controlThread();
    void setArmed(bool _value);
    // void initHomeFrame();
    bool referencePoseReached();
    // void setFlightMode(const std::string& _flight_mode);
    State guessState();
    
    void Quaternion2EulerAngle(const geometry_msgs::Pose::_orientation_type& _q, double& _roll, double& _pitch, double& _yaw);
    bool altimeter_fail(void);
  
    void goHome();

    geometry_msgs::PoseStamped reference_pose_;
    double offset_x;
    double offset_y;
    double offset_xy;

    double offset_x1;
    double offset_y1;

    sensor_msgs::NavSatFix     reference_pose_global_;
    geometry_msgs::TwistStamped reference_vel_;
    geometry_msgs::TwistStamped current_vel_;

    geometry_msgs::PointStamped current_position_;
    sensor_msgs::NavSatFix      current_position_global_;
    geometry_msgs::Vector3Stamped current_linear_velocity_;
    geometry_msgs::Vector3Stamped current_angular_velocity_;
    geometry_msgs::QuaternionStamped current_attitude_;

    geometry_msgs::PoseStamped  cur_pose_;
    // geometry_msgs::PoseStamped  ref_pose_;
    geometry_msgs::TwistStamped cur_vel_;

    std_msgs::UInt8 flight_status_;
    std_msgs::UInt8 display_mode_;
    std_msgs::Float64 current_laser_altitude_;

    // Control
    enum class eControlMode { IDLE, LOCAL_VEL, LOCAL_POSE, GLOBAL_POSE };
    eControlMode control_mode_ = eControlMode::IDLE;
    // bool mavros_has_pose_ = false;
    float position_th_;
    float orientation_th_;
    float vel_factor;
    float vel_factor_max;
    bool laser_altimeter;
    bool self_arming;

    float mpc_xy_vel_max;
    float mpc_z_vel_max_up;
    float mpc_z_vel_max_dn;
    float mc_yawrate_max;

    DjiHistoryBuffer position_error_;
    DjiHistoryBuffer orientation_error_;

    /// Ros Communication
    ros::ServiceClient activation_client_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_local_pos_ref_client_;
    ros::ServiceClient sdk_control_authority_client_;
    ros::ServiceClient drone_task_control_client_;
    ros::ServiceClient mission_waypoint_upload_client;
    ros::ServiceClient mission_waypoint_setSpeed_client;
    ros::ServiceClient mission_waypoint_action_client;

    ros::Publisher flight_control_pub_;
    
    //test publishers
    ros::Publisher lookahead_pub;
    ros::Publisher ref_pose_pub;
    ros::Publisher offset_y_pub;
    
    
    ros::Subscriber position_sub_;
    ros::Subscriber position_global_sub_;
    ros::Subscriber linear_velocity_sub_;
    ros::Subscriber angular_velocity_sub_;
    ros::Subscriber attitude_sub_;
    ros::Subscriber laser_altitude_sub_;
    ros::Subscriber flight_status_sub_;
    ros::Subscriber display_mode_sub_;

    int robot_id_;
    std::string pose_frame_id_;
    // std::string uav_home_frame_id_;
    // tf2_ros::StaticTransformBroadcaster * static_tf_broadcaster_;
    // std::map <std::string, geometry_msgs::TransformStamped> cached_transforms_;
    // Eigen::Vector3d local_start_pos_;
    
    ros::Time last_command_time_;

    std::thread control_thread_;
    double control_thread_frequency_;
    
    bool calling_takeoff = false;
    bool calling_land = false;

    bool activated_ = false;
    bool home_set_ = false;
};

}}	// namespace grvc::ual

#endif // UAV_ABSTRACTION_LAYER_BACKEND_DJI_H
