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
#include <thread>
#include <algorithm>
#include <chrono>

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
        float dt = _dt; // 666 input arg?
    
        float err = mReference - _val;
        // Normalize angular error
        if (is_angular_) {
            if (err > M_PI) {err -= 2*M_PI;}
            if (err < -M_PI) {err += 2*M_PI;}
        }
        accum_err_ += err*dt;
        // Apply anti wind-up 777 Analyze other options
        accum_err_ = std::min(std::max(accum_err_, min_windup_), max_windup_);
    
        // Compute PID
        last_result_ = kp_*err + ki_*accum_err_ + kd_*(err- last_error_)/dt;
        last_error_ = err;
    
        // Saturate signal
        last_result_ = std::min(std::max(last_result_, min_sat_), max_sat_);
        return last_result_;
    }
 
    bool enableRosInterface(std::string _tag) {
        if (ros::isInitialized()) {
            ros::NodeHandle np;
            
            service_kp_   = np.advertiseService(_tag +"/kp",         &PID::serviceKp,      this);
            service_ki_   = np.advertiseService(_tag +"/ki",         &PID::serviceKi,      this);
            service_kd_   = np.advertiseService(_tag +"/kd",         &PID::serviceKd,      this);
            service_sat_  = np.advertiseService(_tag +"/saturation", &PID::serviceKsat,    this);
            service_wind_ = np.advertiseService(_tag +"/windup",     &PID::serviceKwindup, this);
            
            pub_kp_ = np.advertise<std_msgs::Float32>(_tag +"/kp", 1);
            pub_ki_ = np.advertise<std_msgs::Float32>(_tag +"/ki", 1);
            pub_kd_ = np.advertise<std_msgs::Float32>(_tag +"/kd", 1);
            pub_sat_ = np.advertise<std_msgs::Float32>(_tag +"/saturation", 1);
            pub_wind_ = np.advertise<std_msgs::Float32>(_tag +"/windup", 1);
            
            mParamPubThread = std::thread([&](){
                while(ros::ok()){
                    std_msgs::Float32 param;
        
                    param.data = kp_; pub_kp_.publish(param);
                    param.data = ki_; pub_ki_.publish(param);
                    param.data = kd_; pub_kd_.publish(param);
                    param.data = max_sat_; pub_sat_.publish(param);
                    param.data = max_windup_; pub_wind_.publish(param);
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                }
            });
            return true;
        } else {
            return false;
        }
    }
 
    float reference() { return mReference; }
    void reference(float _ref) { mReference = _ref;}

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
 
private:
    bool serviceKp       (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {kp_ = _req.param;};
    bool serviceKi       (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {ki_ = _req.param;};
    bool serviceKd       (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {kd_ = _req.param;};
    bool serviceKsat     (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {min_sat_ = -_req.param; max_sat_ = _req.param;};
    bool serviceKwindup  (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {min_windup_ = -_req.param; max_windup_ = _req.param;};

private:
    bool is_angular_;
    float mReference;
    float kp_, ki_, kd_;
    float min_sat_, max_sat_;
    float min_windup_, max_windup_;
    float last_result_, last_error_, accum_err_;
    ros::ServiceServer service_kp_, service_ki_, service_kd_, service_sat_, service_wind_;
    ros::Publisher pub_kp_, pub_ki_, pub_kd_, pub_sat_, pub_wind_;
    std::thread mParamPubThread;
};

}} // namespace grvc::ual

#endif // UAV_ABSTRACTION_LAYER_PID_H