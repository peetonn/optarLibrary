//
// ros-client.hpp
//
//  Created by Peter Gusev on 5 December 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#ifndef __ros_client_hpp__
#define __ros_client_hpp__

#include <memory>

#include <opencv2/features2d.hpp>
#include <ros/ros.h>
#if __APPLE__
    #include <ros/tf/transform_listener.h>
#else
    #include <tf/transform_listener.h>
#endif
#include "opt_msgs/OptarNtpMessage.h"
#include "types.hpp"

namespace optar
{
namespace ros_components
{

class HeartbeatPublisher;
class RosNtpClient;
class ArPosePublisher;
class OptarCameraPublisher;
class TfListener;

std::string getCrossPlatformWriteableFolder();

typedef std::function<void(const Transform&, int64_t tsUsec)> OnWorldToArTransform;

/**
 * This class is a wrapper for ROS communication behind OPTAR.
 */
class RosClient {
public:
    static std::string TopicNameHeartbeat;
    static std::string TopicNameCentroids;
    static std::string TopicNameSkeletons;
    static std::string TopicNameNtpChat;
    static std::string TopicNameComponentWorld;
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
    static std::shared_ptr<OptarCameraPublisher> createOptarPublisher(std::string);
    static std::shared_ptr<TfListener> createTfListener(std::string, OnWorldToArTransform);

    RosClient(std::shared_ptr<ros::NodeHandle> nh, std::string deviceId);

    std::string getRosTfFrame() const;
    std::string getCameraRosFrame() const;

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

    // returns current ROS time adjusted for server sync
    ros::Time getSyncTime() const;

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
    static std::string getTopicName(std::string deviceId);

    ArPosePublisher(std::shared_ptr<ros::NodeHandle>, std::string, double rate);
    ~ArPosePublisher();

    void publishPose(const Pose& pose);
    const Pose& getLastPose() const { return lastPose_; };

private:
    Pose lastPose_;
    ros::Publisher publisher_;
    ros::Timer timer_;
    double publishRate_;
    int64_t lastPublishTsMs_;
};

class OptarCameraPublisher : public RosClient {
public:
    static std::string getTopicName(std::string deviceId);
    static std::string getDebugTopicName(std::string deviceId);

    OptarCameraPublisher(std::shared_ptr<ros::NodeHandle>, std::string);
    ~OptarCameraPublisher();

    void publish(ros::Time imageTime,
                 const Pose& cameraPose,
                 const CameraIntrinsics& cameraIntrinsics,
                 const cv::Mat& descriptors,
                 const std::vector<cv::KeyPoint>& keyPoints,
                 const cv::Mat& debugImage);

private:
    ros::Publisher publisher_;
    int64_t lastPublishTsMs_;
};

class TfListener : public RosClient {
public:
    TfListener(std::shared_ptr<ros::NodeHandle>, std::string, OnWorldToArTransform);
    ~TfListener();

    bool hasReceivedTf() const { return lastLookupTsMs_ != 0; }
    int64_t getLastTransformTsUsec() const { return lastLookupTsMs_; }
    Transform getLastTransform() const { return lastTransform_; };

private:
    double rate_;
    int64_t lastLookupTsMs_;
    ros::Timer timer_;
    tf::TransformListener listener_;
    Transform lastTransform_;
    OnWorldToArTransform onTransform_;

    void onTimerFire(const ros::TimerEvent& event);
};

}
}

#endif
