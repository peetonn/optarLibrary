//
// ros-client.hpp
//
//  Created by Peter Gusev on 5 December 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#ifndef __ros_client_hpp__
#define __ros_client_hpp__

#include <memory>
#include <ros/ros.h>

#include "opt_msgs/OptarNtpMessage.h"

namespace optar
{
namespace ros_components
{

class HeartbeatPublisher;
class RosNtpClient;
class ArPosePublisher;

/**
 * This class is a wrapper for ROS communication behind OPTAR.
 */
class RosClient {
public:
    static std::string TopicNameHeartbeat;
    static std::string TopicNameCentroids;
    static std::string TopicNameSkeletons;
    static std::string TopicNameNtpChat;
    static std::string TopicNameComponentOptar;
    static std::string TopicNameComponentPose;
    static std::string TopicNameComponentFeatures;
    static std::string TopicNameComponentCamera;
    static std::string TopicNameComponentCameraFrame;

    static void initRos(const std::string &rosMasterUri);
    static bool getIsRosThreadRunning();
    static std::shared_ptr<HeartbeatPublisher> createHeartbeatPublisher(std::string);
    static std::shared_ptr<RosNtpClient> createNtpClient();
    static std::shared_ptr<ArPosePublisher> createPosePublisher(std::string, double rate = 30.);

    RosClient(std::shared_ptr<ros::NodeHandle> nh, std::string deviceId);

    std::string getRosTfFrame() const;

protected:
    std::string deviceId_;
    std::shared_ptr<ros::NodeHandle> nodeHandle_;
};

class HeartbeatPublisher : public RosClient {
public:
    HeartbeatPublisher(std::shared_ptr<ros::NodeHandle>, std::string,
                        double heartbetFrequency = 1);
    ~HeartbeatPublisher();

private:
    double publishFrequency_;
    ros::Publisher publisher_;
    ros::Timer timer_;

    void onTimerFire(const ros::TimerEvent& event);
};

class RosNtpClient : public RosClient {
public:
    RosNtpClient(std::shared_ptr<ros::NodeHandle>, std::string);
    ~RosNtpClient();

    // returns current time (microseconds) adjusted for server sync
    int64_t getSyncTimeUsec() const;

private:
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
    ros::Timer timer_;
    int64_t estimatedTimeDiffUsec_;
    std::string lastRequestId_;
    std::vector<int64_t> timeDiffs_;

    void onTimerFire(const ros::TimerEvent& event);
    void onNtpMessage(const opt_msgs::OptarNtpMessage::ConstPtr&);
    void sendRequest();

};

class ArPosePublisher : public RosClient {
public:
    typedef struct _Pose {
        float posX_, posY_, posZ_;
        float quatX_, quatY_, quatZ_, quatW_;
    } Pose;

    static std::string getTopicName(std::string deviceId);

    ArPosePublisher(std::shared_ptr<ros::NodeHandle>, std::string, double rate);
    ~ArPosePublisher();

    void publishPose(const Pose& pose);

private:
    ros::Publisher publisher_;
    ros::Timer timer_;
    double publishRate_;
    int64_t lastPublishTsMs_;

};

}
}

#endif
