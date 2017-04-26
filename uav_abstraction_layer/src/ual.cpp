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

namespace grvc { namespace ual {

UAL::UAL(int _argc, char** _argv) {
    backend_ = Backend::createBackend(_argc, _argv);
}

void UAL::goToWaypoint(const Waypoint& _wp, bool _blocking) {
    if (_blocking) {
        backend_->threadSafeCall(&Backend::goToWaypoint, _wp);
    } else {
        // Call function on a thread!
        std::thread ([&]() {
            goToWaypoint(_wp, true);  // Kind of recursive!
        }).detach();
    }
}

void UAL::takeOff(double _height, bool _blocking) {
    if (_blocking) {
        backend_->threadSafeCall(&Backend::takeOff, _height);
    } else {
        // Call function on a thread!
        std::thread ([&]() {
            takeOff(_height, true);  // Kind of recursive!
        }).detach();
    }
}

void UAL::land(bool _blocking) {
    if (_blocking) {
        backend_->threadSafeCall(&Backend::land);
    } else {
        // Call function on a thread!
        std::thread ([&]() {
            land(true);  // Kind of recursive!
        }).detach();
    }
}

void UAL::setVelocity(const Velocity& _vel) {
    // Function is non-blocking in backend
    backend_->threadSafeCall(&Backend::setVelocity, _vel);
}

void UAL::setPositionError(const PositionError& _pos_error) {
    // Function is non-blocking in backend
    backend_->threadSafeCall(&Backend::setPositionError, _pos_error);
}

}}	// namespace grvc::ual
