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

namespace grvc { namespace ual {

Backend* Backend::createBackend(int _argc, char** _argv) {
    Backend* be = nullptr;
    be = new BackendMavros(_argc, _argv);
    return be;
}

bool Backend::isIdle() {
    std::lock_guard<std::mutex> lock_guard(running_mutex_);
    return !running_task_;
}

void Backend::abortCurrentTask() {
    std::lock_guard<std::mutex> lock_guard(running_mutex_);
    if (running_task_) { abort_ = true; }
}

bool Backend::goToRunningState() {
    std::lock_guard<std::mutex> lock_guard(running_mutex_);
    if (!running_task_) {
        running_task_ = true;
        return true;
    } else {
        return false;
    }
}

void Backend::goToIdleState() {
    std::lock_guard<std::mutex> lock_guard(running_mutex_);
    running_task_ = false;
    abort_ = false;
}

}}	// namespace grvc::ual
