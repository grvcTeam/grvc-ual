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
#ifndef UAV_ABSTRACTION_LAYER_BACKEND_H
#define UAV_ABSTRACTION_LAYER_BACKEND_H

#include <mutex>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
// #include <nav_msgs/Path.h>

namespace grvc { namespace ual {

typedef geometry_msgs::PoseStamped Pose;
typedef geometry_msgs::PoseStamped Waypoint;
typedef geometry_msgs::TwistStamped Velocity;
typedef geometry_msgs::Vector3Stamped PositionError;
//typedef std_msgs::Float32 TakeOffHeight;

/// Common interface for back-end implementations of ual
class Backend {
public:

    template <typename Callable, typename ... Args>
    bool threadSafeCall(Callable&& _fn, Args&& ... _args) {
        if (!goToRunningState()) { return false; }
        std::bind(_fn, this, std::forward<Args>(_args)...)();
        goToIdleState();
        return true;
    }

    /// Backend is initialized and ready to run tasks?
    virtual bool	isReady() const = 0;
    /// Is it idle?
    virtual bool    isIdle();
    /// Latest pose estimation of the robot
    virtual Pose	pose() const = 0;

    /// Go to the specified waypoint, following a straight line
    /// \param _wp goal waypoint
    virtual void	goToWaypoint(const Waypoint& _wp) = 0;
    /// Follow a list of waypoints, one after another
    // virtual void	trackPath(const Path& _path) = 0;
    /// Perform a take off maneuver
    /// \param _height target height that must be reached to consider the take off complete
    virtual void    takeOff(double _height) = 0;
    /// Land on the current position.
    virtual void	land() = 0;
    /// Set velocities
    /// \param _vel target velocity in world coordinates
    virtual void    setVelocity(const Velocity& _vel) = 0;
    /// Set position error control
    /// \param _pos_error position error in world coordinates
    virtual void	setPositionError(const PositionError& _pos_error) = 0;

    /// Cancel execution of the current task
    virtual void	abortCurrentTask();

    virtual ~Backend() = default; // Ensure proper destructor calling for derived classes

    /// \brief Create an adequate Backend depending on current platform and command arguments
    /// \param _argc number of arguments in _argv
    /// \param _argv command line arguments passed to the program. This arguments will be parsed
    /// and used to select the best fitting implementation of Backend from those available in the
    /// current platform.
    /// \return the newly created Backend. Whoever calls this method, is responsible for eventually
    /// destroying the Backend.
    static Backend* createBackend(int _argc, char** _argv);

protected:
    /// Abort flag
    /// If you want your task to be abortable, check its value periodically
    volatile bool abort_ = false;

    /// Simplest state-machine model: idle/running
    /// With mechanisms to be thread-safe included
    bool goToRunningState();
    void goToIdleState();
    volatile bool running_task_ = false;
    std::mutex running_mutex_;
};

}}	// namespace grvc::ual

#endif // UAV_ABSTRACTION_LAYER_BACKEND_H
