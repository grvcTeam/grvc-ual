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
#include <ros/ros.h>

namespace grvc { namespace ual {

UAL::UAL(int _argc, char** _argv) {
    backend_ = Backend::createBackend(_argc, _argv);
}

UAL::~UAL() {
    if (!backend_->isIdle()) { backend_->abort(); }
    if (running_thread_.joinable()) running_thread_.join();
}

bool UAL::goToWaypoint(const Waypoint& _wp, bool _blocking) {
    // Check required state
    if (state_ != FLYING) {
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(); }

    if (_blocking) {
        if (!backend_->threadSafeCall(&Backend::goToWaypoint, _wp)) {
            ROS_INFO("Blocking goToWaypoint rejected!");
            return false;
        }
    } else {
        if (running_thread_.joinable()) running_thread_.join();
        // Call function on a thread:
        running_thread_ = std::thread ([this, _wp]() {
            if (!this->backend_->threadSafeCall(&Backend::goToWaypoint, _wp)) {
                ROS_INFO("Non-blocking goToWaypoint rejected!");
            }
        });
    }
    return true;
}

bool UAL::takeOff(double _height, bool _blocking) {
    // Check required state
    if (state_ != LANDED) {
        return false;
    }
    state_ = TAKING_OFF;

    if (_blocking) {
        if (!backend_->threadSafeCall(&Backend::takeOff, _height)) {
            ROS_INFO("Blocking takeOff rejected!");
            return false;
        }
        state_ = FLYING;
    } else {
        if (running_thread_.joinable()) running_thread_.join();
        // Call function on a thread:
        running_thread_ = std::thread ([this, _height]() {
            if (!this->backend_->threadSafeCall(&Backend::takeOff, _height)) {
                ROS_INFO("Non-blocking takeOff rejected!");
            }
            this->state_ = FLYING;
        });
    }
    return true;
}

bool UAL::land(bool _blocking) {
    // Check required state
    if (state_ != FLYING) {
        return false;
    }
    state_ = LANDING;

    if (_blocking) {
        if (!backend_->threadSafeCall(&Backend::land)) {
            ROS_INFO("Blocking land rejected!");
            return false;
        }
        state_ = LANDED;
    } else {
        if (running_thread_.joinable()) running_thread_.join();
        // Call function on a thread:
        running_thread_ = std::thread ([this]() {
            if (!this->backend_->threadSafeCall(&Backend::land)) {
                ROS_INFO("Non-blocking land rejected!");
            }
            this->state_ = LANDED;
        });
    }
    return true;
}

bool UAL::setVelocity(const Velocity& _vel) {
    // Check required state
    if (state_ != FLYING) {
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(); }

    // Function is non-blocking in backend TODO: non-thread-safe-call?
    backend_->threadSafeCall(&Backend::setVelocity, _vel);
    return true;
}

bool UAL::setPositionError(const PositionError& _pos_error) {
    // Check required state
    if (state_ != FLYING) {
        return false;
    }
    // Override any previous FLYING function
    if (!backend_->isIdle()) { backend_->abort(); }

    // Function is non-blocking in backend TODO: non-thread-safe-call?
    backend_->threadSafeCall(&Backend::setPositionError, _pos_error);
    return true;
}

}}	// namespace grvc::ual
