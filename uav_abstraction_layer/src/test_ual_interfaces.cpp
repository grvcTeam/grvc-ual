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

int main(int _argc, char** _argv) {

    grvc::utils::ArgumentParser args(_argc, _argv);

    grvc::ual::UAL ual(args);
    while (!ual.isReady() && ros::ok()) {
        std::cout << "UAL not ready!" << std::endl;
        sleep(1);
    }

    // Start server if explicitly asked:
    std::string server_mode = args.getArgument<std::string>("ual_server", "off");
    if (server_mode == "on") {
        std::cout << "UAL server is now available" << std::endl;
        // Do nothing
        while (ros::ok()) { sleep(1); }
        return 0;
    }

    // Else, use class interface to perform a simple mission:
    std::cout << "Mission using UAL object" << std::endl;
    // Define flight level and take off
	double flight_level = 10.0;
    ual.takeOff(flight_level);

    // Define path (ugly due to lack of constructor)
    double square_lenght = 10.0;
    std::list<grvc::ual::Waypoint> path;
    grvc::ual::Waypoint waypoint;
    waypoint.header.frame_id = "map";
    waypoint.pose.position.x = +0.5*square_lenght;
    waypoint.pose.position.y = +0.5*square_lenght;
    waypoint.pose.position.z = flight_level;
    path.push_back(waypoint);
    waypoint.pose.position.x = -0.5*square_lenght;
    waypoint.pose.position.y = +0.5*square_lenght;
    path.push_back(waypoint);
    waypoint.pose.position.x = -0.5*square_lenght;
    waypoint.pose.position.y = -0.5*square_lenght;
    path.push_back(waypoint);
    waypoint.pose.position.x = +0.5*square_lenght;
    waypoint.pose.position.y = -0.5*square_lenght;
    path.push_back(waypoint);

    std::cout << "Blocking version of goToWaypoint" << std::endl;
    for (auto p : path) {
        std::cout << "Waypoint: " << p.pose.position.x << ", " << \
            p.pose.position.y << ", " << p.pose.position.z << ", frame_id: " << p.header.frame_id << std::endl;
        ual.goToWaypoint(p);
        std::cout << "Arrived!" << std::endl;
    }

    std::cout << "Non-blocking version of goToWaypoint" << std::endl;
    for (auto p : path) {
        std::cout << "Waypoint: " << p.pose.position.x << ", " << \
            p.pose.position.y << ", " << p.pose.position.z << ", frame_id: " << p.header.frame_id << std::endl;
        ual.goToWaypoint(p, false);
        do {
            grvc::ual::Velocity v = ual.velocity();
            std::cout << "v = [" << v.twist.linear.x << ", " << v.twist.linear.y << ", " \
                << v.twist.linear.z << "]" << std::endl;
            std::cout << "Waiting" << std::endl;
            sleep(1);
        } while (!ual.isIdle());
        std::cout << "Arrived!" << std::endl;
    }

    // Land
    ual.land();

    return 0;
}
