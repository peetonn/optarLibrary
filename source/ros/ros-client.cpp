//
// ros-client.cpp
//
//  Created by Peter Gusev on 5 December 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#include "ros-client.hpp"

#include <ros/ros.h>
#include <thread>

#include "../logging.hpp"

using namespace std;
using namespace optar;
using namespace ros;


bool RunRos = false;
thread RosThread;
shared_ptr<NodeHandle> RosNodeHandle;
void spinRos();

//******************************************************************************
void
RosClient::initRos(const std::string &rosMasterUri)
{
    if (RunRos)
    {
        OLOG_INFO("Stopping ROS thread...");
        ros::shutdown();
        RunRos = false;
        RosThread.join();
    }

    static map<string,string> remappings;
    remappings["__master"] = rosMasterUri;

    OLOG_INFO("Initializing ROS (master URI {})...", rosMasterUri);
    ros::init(remappings, "optar_client",
              ros::init_options::NoRosout |
              ros::init_options::AnonymousName |
              ros::init_options::NoSigintHandler);

    OLOG_INFO("Starting ROS node...");
    static once_flag onceFlag;;
    call_once(onceFlag, [](){
        RosNodeHandle = make_shared<NodeHandle>();
    });
    RosThread = thread(spinRos);
}

//******************************************************************************
void spinRos()
{
    OLOG_INFO("Starting ROS thread...");

    RunRos = true;
    while (ros::ok())
    {
        ros::spin();
    }

    OLOG_INFO("ROS thread stopped.");
}
