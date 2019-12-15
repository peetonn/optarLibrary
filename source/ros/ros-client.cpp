//
// ros-client.cpp
//
//  Created by Peter Gusev on 5 December 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#include "ros-client.hpp"

#include <ros/ros.h>
#include <uuid/uuid.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <thread>
#include <numeric>
#include <ctime>

#include "../logging.hpp"
#include "../utils/clock.hpp"

using namespace std;
using namespace optar;
using namespace optar::ros_components;
using namespace ros;

bool RunRos = false;
thread RosThread;
shared_ptr<NodeHandle> RosNodeHandle;
void spinRos();
string getRandomUuid();

std::string RosClient::TopicNameHeartbeat = "/optar/heartbeats";
std::string RosClient::TopicNameCentroids = "/tracker/tracks_smoothed";
std::string RosClient::TopicNameSkeletons = "/tracker/skeleton_tracks";
std::string RosClient::TopicNameNtpChat = "/optar/ntp_chat";
std::string RosClient::TopicNameComponentOptar = "optar";
std::string RosClient::TopicNameComponentPose = "pose";
std::string RosClient::TopicNameComponentFeatures = "features";
std::string RosClient::TopicNameComponentCamera = "camera";
std::string RosClient::TopicNameComponentCameraFrame = "_camera_frame";

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

bool
RosClient::getIsRosThreadRunning()
{
    return RunRos;
}

shared_ptr<HeartbeatPublisher>
RosClient::createHeartbeatPublisher(std::string deviceId)
{
    return make_shared<HeartbeatPublisher>(RosNodeHandle, deviceId);
}

shared_ptr<RosNtpClient>
RosClient::createNtpClient()
{
    return make_shared<RosNtpClient>(RosNodeHandle, ""); // device id is not used
}

shared_ptr<ArPosePublisher>
RosClient::createPosePublisher(std::string deviceId, double rate)
{
    return make_shared<ArPosePublisher>(RosNodeHandle, deviceId, rate);
}

RosClient::RosClient(shared_ptr<NodeHandle> nh,string deviceId)
: nodeHandle_(nh)
, deviceId_(deviceId)
{}

string
RosClient::getRosTfFrame() const
{
    return deviceId_ + "__world_filtered";
}

//******************************************************************************
HeartbeatPublisher::HeartbeatPublisher(shared_ptr<NodeHandle> nh,
    string deviceId, double heartbeatFrequency)
: RosClient(nh, deviceId)
, publishFrequency_(heartbeatFrequency)
, publisher_(nodeHandle_->advertise<std_msgs::String>(RosClient::TopicNameHeartbeat, 1))
, timer_(nodeHandle_->createTimer(ros::Duration(1/publishFrequency_), &HeartbeatPublisher::onTimerFire, this))
{
    OLOG_DEBUG("Heartbeat publisher started: device id {} frequency {}",
        deviceId_, publishFrequency_);
}

HeartbeatPublisher::~HeartbeatPublisher()
{}

void
HeartbeatPublisher::onTimerFire(const ros::TimerEvent& event)
{
    std_msgs::String msg;
    msg.data = deviceId_;
    publisher_.publish(msg);

    // OLOG_DEBUG("Heartbeat published.");
}

//******************************************************************************
#define NTP_SYNC_INTERVAL 5
#define NTP_TIME_DIFF_QSIZE 100
RosNtpClient::RosNtpClient(shared_ptr<NodeHandle> nh, string deviceId)
: RosClient(nh, deviceId)
, publisher_(nodeHandle_->advertise<opt_msgs::OptarNtpMessage>(RosClient::TopicNameNtpChat, 10))
, subscriber_(nodeHandle_->subscribe(RosClient::TopicNameNtpChat, 1,
    &RosNtpClient::onNtpMessage, this))
, timer_(nodeHandle_->createTimer(ros::Duration(NTP_SYNC_INTERVAL), &RosNtpClient::onTimerFire, this))
{}

RosNtpClient::~RosNtpClient()
{}

int64_t
RosNtpClient::getSyncTimeUsec() const
{
    int64_t nowUsec = Time::now().toNSec()/1000;
    return nowUsec + estimatedTimeDiffUsec_;
}

void
RosNtpClient::onTimerFire(const TimerEvent& event)
{
    sendRequest();
}

void
RosNtpClient::onNtpMessage(const opt_msgs::OptarNtpMessage::ConstPtr& msg)
{
    using namespace opt_msgs;

    if (msg->id == lastRequestId_ && msg->type == OptarNtpMessage::REPLY)
    {
        OLOG_DEBUG("Received NTP message: id {}", msg->id);

        int64_t now = Time::now().toNSec();
        int64_t clientReceiveTimeUsec = now / 1000;
        int64_t clientRequestTimeUsec = msg->clientRequestTime.toNSec() / 1000;
        int64_t serverTimeUsec = msg->serverTime.toNSec() / 1000;

        OLOG_DEBUG("NTP client req {}us recv {}us server reply {}us",
            clientRequestTimeUsec, clientReceiveTimeUsec, serverTimeUsec);

        int64_t timeDiffUsec = ((serverTimeUsec - clientRequestTimeUsec) +
                                (serverTimeUsec - clientReceiveTimeUsec)) / 2;

        if (timeDiffs_.size() >= NTP_TIME_DIFF_QSIZE)
            timeDiffs_.erase(timeDiffs_.begin());

        timeDiffs_.push_back(timeDiffUsec);
        int64_t sum = accumulate(timeDiffs_.begin(), timeDiffs_.end(), 0);
        estimatedTimeDiffUsec_ = sum / (int64_t)timeDiffs_.size();

        OLOG_DEBUG("NTP time diff est {}us", estimatedTimeDiffUsec_);
        lastRequestId_ = "";
    }
}

void
RosNtpClient::sendRequest()
{
    using namespace opt_msgs;
    OptarNtpMessagePtr request(new OptarNtpMessage);

    request->type = OptarNtpMessage::QUERY;
    request->clientRequestTime = Time::now();
    request->id = getRandomUuid();

    publisher_.publish(request);
    lastRequestId_ = request->id;

    OLOG_DEBUG("Sent NTP request: id {}", lastRequestId_);
}

//******************************************************************************
ArPosePublisher::ArPosePublisher(shared_ptr<NodeHandle> nh, string deviceId,
                                 double publishRate)
: RosClient(nh, deviceId)
, publisher_(nodeHandle_->advertise<geometry_msgs::PoseStamped>(ArPosePublisher::getTopicName(deviceId), 10))
, publishRate_(publishRate)
, lastPublishTsMs_(0)
{
}

ArPosePublisher::~ArPosePublisher() {}

string
ArPosePublisher::getTopicName(std::string deviceId)
{
    return RosClient::TopicNameComponentOptar + "/" +
            deviceId + "/" +
            RosClient::TopicNameComponentPose;
}

void
ArPosePublisher::publishPose(const Pose& pose)
{
    int64_t now = clock::millisecondTimestamp();

    if (double(now - lastPublishTsMs_) >= 1000./publishRate_)
    {
        using namespace geometry_msgs;
        PoseStampedPtr msg(new PoseStamped);

        msg->pose.position.x = pose.posX_;
        msg->pose.position.y = pose.posY_;
        msg->pose.position.z = pose.posZ_;
        msg->pose.orientation.x = pose.quatX_;
        msg->pose.orientation.y = pose.quatY_;
        msg->pose.orientation.z = pose.quatZ_;
        msg->pose.orientation.w = pose.quatW_;

        msg->header.frame_id = getRosTfFrame();

        publisher_.publish(msg);
        lastPublishTsMs_ = now;

        OLOG_DEBUG("Published AR pose: [{:.2f} {:.2f} {:.2f}] [{:.2f} {:.2f} {:.2f} {:.2f}]",
            pose.posX_, pose.posY_, pose.posZ_,
            pose.quatX_, pose.quatY_, pose.quatZ_, pose.quatW_);
    }
}

//******************************************************************************
string getRandomUuid()
{
    // uuid_t uuid;
    // uuid_generate_random(uuid);
    // stringstream ss;
    // for (int i = 0; i < sizeof(uuid); ++i)
    //     ss << hex << uuid[i];
    // return ss.str();
    srand(time(nullptr));
    stringstream ss;
    ss << clock::nanosecondTimestamp() << "-" << rand();
    return ss.str();
}

void spinRos()
{
    OLOG_INFO("Starting ROS thread...");

    RunRos = true;
    while (ros::ok())
    {
        ros::spin();
    }

    OLOG_INFO("ROS thread stopped.");
    RunRos = false;
}
