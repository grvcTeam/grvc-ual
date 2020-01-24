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
};

class PID {
public:
    PID(float _kp, float _ki, float _kd,
        float _minSat = std::numeric_limits<float>::min(),
        float _maxSat = std::numeric_limits<float>::max(),
        float _minWind = std::numeric_limits<float>::min(),
        float _maxWind = std::numeric_limits<float>::max()) {

        mKp = _kp;
        mKi = _ki;
        mKd = _kd;
        mMinSat = _minSat;
        mMaxSat = _maxSat;
        mWindupMin = _minWind;
        mWindupMax = _maxWind;    
    }

    PID(PIDParams params)
        : PID(params.kp, params.ki, params.kd,
            params.min_sat, params.max_sat,
            params.min_wind, params.max_wind) {}

    float update(float _val, float _incT) {
        float dt = _incT; // 666 input arg?
    
        float err = mReference - _val;
        mAccumErr += err*dt;
        // Apply anti wind-up 777 Analyze other options
        mAccumErr = std::min(std::max(mAccumErr, mWindupMin), mWindupMax);
    
        // Compute PID
        mLastResult = mKp*err + mKi*mAccumErr + mKd*(err- mLastError)/dt;
        mLastError = err;
    
        // Saturate signal
        mLastResult = std::min(std::max(mLastResult, mMinSat), mMaxSat);
        mLastResult *=mBouncingFactor;
        mBouncingFactor *=2.0;
        mBouncingFactor = mBouncingFactor > 1.0? 1.0:mBouncingFactor;
        return mLastResult;
    }
 
    bool enableRosInterface(std::string _tag) {
        if (ros::isInitialized()) {
            ros::NodeHandle np;
            
            mServiceKp   = np.advertiseService(_tag +"/kp",         &PID::serviceKp,      this);
            mServiceKi   = np.advertiseService(_tag +"/ki",         &PID::serviceKi,      this);
            mServiceKd   = np.advertiseService(_tag +"/kd",         &PID::serviceKd,      this);
            mServiceSat  = np.advertiseService(_tag +"/saturation", &PID::serviceKsat,    this);
            mServiceWind = np.advertiseService(_tag +"/windup",     &PID::serviceKwindup, this);
            
            mPubKp = np.advertise<std_msgs::Float32>(_tag +"/kp", 1);
            mPubKi = np.advertise<std_msgs::Float32>(_tag +"/ki", 1);
            mPubKd = np.advertise<std_msgs::Float32>(_tag +"/kd", 1);
            mPubSat = np.advertise<std_msgs::Float32>(_tag +"/saturation", 1);
            mPubWind = np.advertise<std_msgs::Float32>(_tag +"/windup", 1);
            
            mParamPubThread = std::thread([&](){
                while(ros::ok()){
                    std_msgs::Float32 param;
        
                    param.data = mKp; mPubKp.publish(param);
                    param.data = mKi; mPubKi.publish(param);
                    param.data = mKd; mPubKd.publish(param);
                    param.data = mMaxSat; mPubSat.publish(param);
                    param.data = mWindupMax; mPubWind.publish(param);
                    std::this_thread::sleep_for(std::chrono::seconds(3));
                }
            });
            return true;
        } else {
            return false;
        }
    }
 
    float reference() { return mReference; }
    void reference(float _ref) { mReference = _ref; mAccumErr = 0; mLastError = 0; mLastResult = 0; mBouncingFactor = 0.1;}
 
    float kp() const { return mKp; }
    float ki() const { return mKi; }
    float kd() const { return mKd; }
 
    void kp(float _kp) { mKp = _kp; }
    void ki(float _ki) { mKi = _ki; }
    void kd(float _kd) { mKd = _kd; }
 
    void setSaturations(float _min, float _max) { mMinSat = _min; mMaxSat = _max; }
    void getSaturations(float _min, float _max) { _min = mMinSat; _max = mMaxSat; }
 
    void setWindupTerms(float _min, float _max) { mWindupMin = _min; mWindupMax = _max; }
    void getWindupTerms(float _min, float _max) { _min = mWindupMin; _max = mWindupMax; }
 
private:
    bool serviceKp       (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {mKp = _req.param;};
    bool serviceKi       (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {mKi = _req.param;};
    bool serviceKd       (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {mKd = _req.param;};
    bool serviceKsat     (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {mMinSat = -_req.param; mMaxSat = _req.param;};
    bool serviceKwindup  (uav_abstraction_layer::Float32Param::Request &_req, uav_abstraction_layer::Float32Param::Response &_res)  {mWindupMin = -_req.param; mWindupMax = _req.param;};

private:
    float mReference;
    float mKp, mKi, mKd;
    float mMinSat, mMaxSat;
    float mWindupMin, mWindupMax;
    float mLastResult, mLastError, mAccumErr;
    ros::ServiceServer mServiceKp, mServiceKi, mServiceKd, mServiceSat, mServiceWind;
    ros::Publisher mPubKp, mPubKi, mPubKd, mPubSat, mPubWind;
    double mBouncingFactor = 0.1;
    std::thread mParamPubThread;
};

}} // namespace grvc::ual

#endif // UAV_ABSTRACTION_LAYER_PID_H