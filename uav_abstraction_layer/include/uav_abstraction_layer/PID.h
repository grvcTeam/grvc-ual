//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 ViGUS University of Seville
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
//  and associated documentation files (the "Software"), to deal in the Software without restriction, 
//  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial 
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------
// Complete and independent version available in https://github.com/Bardo91/ros_uav_abstraction_layer
//---------------------------------------------------------------------------------------------------------------------
#ifndef UAV_ABSTRACTION_LAYER_PID_H
#define UAV_ABSTRACTION_LAYER_PID_H

#include <limits>
#include <ros/ros.h>
#include <uav_abstraction_layer/Float32Param.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <thread>
#include <algorithm>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace grvc { namespace ual {

struct PIDParams {
    float kp;
    float ki;
    float kd;
    float min_sat;
    float max_sat;
    float min_wind;
    float max_wind;
    bool is_angular = false;
};

class PID {
public:
    PID(float _kp, float _ki, float _kd,
        float _min_sat = std::numeric_limits<float>::min(),
        float _max_sat = std::numeric_limits<float>::max(),
        float _min_windup = std::numeric_limits<float>::min(),
        float _max_windup = std::numeric_limits<float>::max(),
        bool _is_angular = false) {

        kp_ = _kp;
        ki_ = _ki;
        kd_ = _kd;
        min_sat_ = _min_sat;
        max_sat_ = _max_sat;
        min_windup_ = _min_windup;
        max_windup_ = _max_windup;
        is_angular_ = _is_angular;
    }

    PID(PIDParams params)
        : PID(params.kp, params.ki, params.kd, params.min_sat, params.max_sat,
            params.min_wind, params.max_wind, params.is_angular) {}

    float update(float _val, float _dt) {
        float err = reference_ - _val;
        
        return updateError(err,_dt);
    }

    float updateError(float _err, float _dt) {
        // Normalize angular error
        if (is_angular_) {
            if (_err > M_PI) {_err -= 2*M_PI;}
            if (_err < -M_PI) {_err += 2*M_PI;}
        }
        accum_err_ += _err*_dt;
        // Apply anti wind-up 777 Analyze other options
        accum_err_ = std::min(std::max(accum_err_, min_windup_), max_windup_);
    
        // Compute PI
        last_result_ = kp_*_err + ki_*accum_err_;
        // Add D term unless is NaN
        float d_term = kd_*(_err- last_error_)/_dt;
        if (!std::isnan(d_term)) {
            last_result_ += d_term;
        }
        last_error_ = _err;
    
        // Saturate signal
        last_result_ = std::min(std::max(last_result_, min_sat_), max_sat_);
        return last_result_;
    }

    ~PID() {
        disableRosInterface();
    }
 
    bool enableRosInterface(std::string _tag) {
        if (ros::isInitialized()) {
            ros::NodeHandle np;
            tag_ = _tag;
            
            service_kp_   = np.advertiseService(_tag +"/kp",         &PID::serviceKp,      this);
            service_ki_   = np.advertiseService(_tag +"/ki",         &PID::serviceKi,      this);
            service_kd_   = np.advertiseService(_tag +"/kd",         &PID::serviceKd,      this);
            service_sat_  = np.advertiseService(_tag +"/saturation", &PID::serviceKsat,    this);
            service_wind_ = np.advertiseService(_tag +"/windup",     &PID::serviceKwindup, this);
            service_save_params_ = np.advertiseService(_tag +"/save_params", &PID::saveParams, this);
            
            pub_kp_ = np.advertise<std_msgs::Float32>(_tag +"/kp", 1);
            pub_ki_ = np.advertise<std_msgs::Float32>(_tag +"/ki", 1);
            pub_kd_ = np.advertise<std_msgs::Float32>(_tag +"/kd", 1);
            pub_sat_ = np.advertise<std_msgs::Float32>(_tag +"/saturation", 1);
            pub_wind_ = np.advertise<std_msgs::Float32>(_tag +"/windup", 1);

            pub_reference_ = np.advertise<std_msgs::Float32>(_tag +"/reference", 1);
            pub_last_value_ = np.advertise<std_msgs::Float32>(_tag +"/system_output", 1);
            pub_last_result_ = np.advertise<std_msgs::Float32>(_tag +"/control_output", 1);
            pub_last_error_ = np.advertise<std_msgs::Float32>(_tag +"/error", 1);
            pub_accum_err_ = np.advertise<std_msgs::Float32>(_tag +"/accum_err", 1);

            is_ros_enabled_ = true;

            param_pub_thread_ = std::thread([&](){
                while(ros::ok() && is_ros_enabled_){
                    std_msgs::Float32 param;

                    param.data = kp_; pub_kp_.publish(param);
                    param.data = ki_; pub_ki_.publish(param);
                    param.data = kd_; pub_kd_.publish(param);
                    param.data = max_sat_; pub_sat_.publish(param);
                    param.data = max_windup_; pub_wind_.publish(param);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            });

            status_pub_thread_ = std::thread([&](){
                while(ros::ok() && is_ros_enabled_){
                    std_msgs::Float32 value;

                    value.data = reference_; pub_reference_.publish(value);
                    value.data = last_value_; pub_last_value_.publish(value);
                    value.data = last_result_; pub_last_result_.publish(value);
                    value.data = last_error_; pub_last_error_.publish(value);
                    value.data = accum_err_; pub_accum_err_.publish(value);
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            });

            return true;
        } else {
            return false;
        }
    }

    void disableRosInterface() {
        if (is_ros_enabled_) {
            // Close threads
            is_ros_enabled_ = false;
            if(param_pub_thread_.joinable()) {param_pub_thread_.join();}
            if(status_pub_thread_.joinable()) {status_pub_thread_.join();}
            // Shutdown service servers
            service_kp_.shutdown();
            service_ki_.shutdown();
            service_kd_.shutdown();
            service_sat_.shutdown();
            service_wind_.shutdown();
            service_save_params_.shutdown();
            // Shutdown params publishers
            pub_kp_.shutdown();
            pub_ki_.shutdown();
            pub_kd_.shutdown();
            pub_sat_.shutdown();
            pub_wind_.shutdown();
            // Shutdown status publishers
            pub_reference_.shutdown();
            pub_last_value_.shutdown();
            pub_last_result_.shutdown();
            pub_last_error_.shutdown();
            pub_accum_err_.shutdown();
        }
    }
 
    float reference() { return reference_; }
    void reference(float _ref) { reference_ = _ref;}

    void reset() { accum_err_ = 0; last_error_ = 0; last_result_ = 0;}
 
    float kp() const { return kp_; }
    float ki() const { return ki_; }
    float kd() const { return kd_; }
 
    void kp(float _kp) { kp_ = _kp; reset();}
    void ki(float _ki) { ki_ = _ki; reset();}
    void kd(float _kd) { kd_ = _kd; reset();}
 
    void setSaturations(float _min, float _max) { min_sat_ = _min; max_sat_ = _max; reset();}
    void getSaturations(float _min, float _max) { _min = min_sat_; _max = max_sat_; }
 
    void setWindupTerms(float _min, float _max) { min_windup_ = _min; max_windup_ = _max; reset();}
    void getWindupTerms(float _min, float _max) { _min = min_windup_; _max = max_windup_; }

    YAML::Node getParamsInYaml() {
        YAML::Node yaml_node;
        yaml_node["kp"] = kp_;
        yaml_node["ki"] = ki_;
        yaml_node["kd"] = kd_;
        yaml_node["min_sat"] = min_sat_;
        yaml_node["max_sat"] = max_sat_;
        yaml_node["min_wind"] = min_windup_;
        yaml_node["max_wind"] = max_windup_;

        return yaml_node;
    }
 
private:
    bool serviceKp      (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {kp_ = _req.param; return true;};
    bool serviceKi      (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {ki_ = _req.param; return true;};
    bool serviceKd      (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {kd_ = _req.param; return true;};
    bool serviceKsat    (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {min_sat_ = -_req.param; max_sat_ = _req.param; return true;};
    bool serviceKwindup (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {min_windup_ = -_req.param; max_windup_ = _req.param; return true;};

    bool saveParams (std_srvs::Trigger::Request &_req, std_srvs::Trigger::Response &_res) {
        YAML::Node parent_node;
        parent_node[tag_] = getParamsInYaml();

        std::string file_name = tag_ + ".yaml";
        std::replace( file_name.begin(), file_name.end(), '/', '_' );
        ROS_INFO("Saving params in file: %s", file_name.c_str());
        std::ofstream fout(file_name.c_str());
        fout << parent_node;
        fout << std::endl;
        fout.close();
        _res.success = true;
        return true;
    }

private:
    bool is_angular_;
    float reference_;
    float kp_, ki_, kd_;
    float min_sat_, max_sat_;
    float min_windup_, max_windup_;
    float last_value_, last_result_, last_error_, accum_err_;
    ros::ServiceServer service_kp_, service_ki_, service_kd_, service_sat_, service_wind_, service_save_params_;
    ros::Publisher pub_kp_, pub_ki_, pub_kd_, pub_sat_, pub_wind_;
    ros::Publisher pub_reference_, pub_last_value_, pub_last_result_, pub_last_error_, pub_accum_err_;
    std::thread param_pub_thread_, status_pub_thread_;
    std::string tag_;
    bool is_ros_enabled_ = false;
};

}} // namespace grvc::ual

#endif // UAV_ABSTRACTION_LAYER_PID_H