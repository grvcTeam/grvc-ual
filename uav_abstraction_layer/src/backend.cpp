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
#include <uav_abstraction_layer/backend.h>
#include <uav_abstraction_layer/backend_mavros.h>
#include <uav_abstraction_layer/backend_light.h>
#include <uav_abstraction_layer/backend_dummy.h>

namespace grvc { namespace ual {

Backend::Backend() {
    // Make communications spin!
    spin_thread_ = std::thread([this]() {
        ros::MultiThreadedSpinner spinner(2); // Use 2 threads
        spinner.spin();
    });
}

Backend* Backend::createBackend() {
    Backend* be = nullptr;
    // Decide backend from arguments:
    ros::NodeHandle nh("~");
    std::string selected_backend;
    nh.param<std::string>("backend", selected_backend, "mavros");
    if (selected_backend == "mavros") {
        be = new BackendMavros();
    }
    else if (selected_backend == "light") {
        be = new BackendLight();
    }
    else if (selected_backend == "dummy") {
        be = new BackendDummy();
    }
    return be;
}

bool Backend::isIdle() {
    return !running_task_;
}

void Backend::abort(bool _freeze) {
    // Block until end of task
    while (running_task_) {
        abort_ = true;
        freeze_ = _freeze;
        std::this_thread::yield();
     }
    // Reset flag
    abort_ = false;
    freeze_ = false;
}

}}	// namespace grvc::ual
