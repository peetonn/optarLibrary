//
// ros-client.cpp
//
//  Created by Peter Gusev on 5 December 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#include "ros-client.hpp"

#include <ctime>
#include <numeric>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include <uuid/uuid.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <opt_msgs/ArcoreCameraFeatures.h>

#include "../logging.hpp"
#include "../utils/clock.hpp"

using namespace std;
using namespace optar;
using namespace optar::ros_components;
using namespace ros;

#define ROS_PUBLISHER_QSIZE 10

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

shared_ptr<OptarCameraPublisher>
RosClient::createOptarPublisher(std::string deviceId)
{
    return make_shared<OptarCameraPublisher>(RosNodeHandle, deviceId);
}

RosClient::RosClient(shared_ptr<NodeHandle> nh,string deviceId)
: nodeHandle_(nh)
, deviceId_(deviceId)
{}

string
RosClient::getRosTfFrame() const
{
    return deviceId_ + "_world_filtered";
}

string
RosClient::getCameraRosFrame() const
{
    return deviceId_ + RosClient::TopicNameComponentCameraFrame;
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
, publisher_(nodeHandle_->advertise<opt_msgs::OptarNtpMessage>(RosClient::TopicNameNtpChat, ROS_PUBLISHER_QSIZE))
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

ros::Time
RosNtpClient::getSyncTime() const
{
    int64_t syncTimeUsec = getSyncTimeUsec();
    int64_t sec = (syncTimeUsec / 1000000);
    int64_t nsec = (syncTimeUsec % 1000000)*1000;

    return ros::Time(int32_t(sec), int32_t(nsec));
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
, publisher_(nodeHandle_->advertise<geometry_msgs::PoseStamped>(ArPosePublisher::getTopicName(deviceId), ROS_PUBLISHER_QSIZE))
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
    lastPose_ = pose;

    if (double(now - lastPublishTsMs_) >= 1000./publishRate_)
    {
        using namespace geometry_msgs;
        PoseStampedPtr msg(new PoseStamped);

        msg->pose.position.x = pose.position_.x_;
        msg->pose.position.y = pose.position_.y_;
        msg->pose.position.z = pose.position_.z_;
        msg->pose.orientation.x = pose.rotation_.x_;
        msg->pose.orientation.y = pose.rotation_.y_;
        msg->pose.orientation.z = pose.rotation_.z_;
        msg->pose.orientation.w = pose.rotation_.w_;

        msg->header.frame_id = getRosTfFrame();

        publisher_.publish(msg);
        lastPublishTsMs_ = now;

        OLOG_DEBUG("Published AR pose: [{:.2f} {:.2f} {:.2f}] [{:.2f} {:.2f} {:.2f} {:.2f}]",
            pose.position_.x_, pose.position_.y_, pose.position_.z_,
            pose.rotation_.x_, pose.rotation_.y_, pose.rotation_.z_, pose.rotation_.w_);
    }
}

//******************************************************************************
string
OptarCameraPublisher::getTopicName(string deviceId)
{
    return RosClient::TopicNameComponentOptar + "/" +
            deviceId + "/" +
            RosClient::TopicNameComponentFeatures;
}

string
OptarCameraPublisher::getDebugTopicName(string deviceId)
{
    return RosClient::TopicNameComponentOptar + "/" +
            deviceId + "/debug";
}

OptarCameraPublisher::OptarCameraPublisher(shared_ptr<NodeHandle> nh, string deviceId)
: RosClient(nh, deviceId)
, lastPublishTsMs_(0)
, publisher_(nodeHandle_->advertise<opt_msgs::ArcoreCameraFeatures>(OptarCameraPublisher::getTopicName(deviceId), ROS_PUBLISHER_QSIZE))
{
}

OptarCameraPublisher::~OptarCameraPublisher()
{
}

void
OptarCameraPublisher::publish(ros::Time imageTime,
                              const Pose& cameraPose,
                              const CameraIntrinsics& cameraIntrinsics,
                              const cv::Mat& descriptors,
                              const vector<cv::KeyPoint>& keyPoints,
                              const cv::Mat& debugImage)
{
    using namespace opt_msgs;
    ArcoreCameraFeaturesPtr msg(new ArcoreCameraFeatures);

    // msg->header.Update();
    msg->header.stamp = imageTime;
    msg->header.frame_id = getCameraRosFrame();

    msg->mobileFramePose.position.x = cameraPose.position_.x_;
    msg->mobileFramePose.position.y = cameraPose.position_.y_;
    msg->mobileFramePose.position.z = cameraPose.position_.z_;
    msg->mobileFramePose.orientation.x = cameraPose.rotation_.x_;
    msg->mobileFramePose.orientation.y = cameraPose.rotation_.y_;
    msg->mobileFramePose.orientation.z = cameraPose.rotation_.z_;
    msg->mobileFramePose.orientation.w = cameraPose.rotation_.w_;

    msg->focal_length_x_px = cameraIntrinsics.focalLengthX_;
    msg->focal_length_y_px = cameraIntrinsics.focalLengthY_;
    msg->image_width_px = cameraIntrinsics.imageWidth_;
    msg->image_height_px = cameraIntrinsics.imageHeight_;
    msg->principal_point_x_px = cameraIntrinsics.principalPointX_;
    msg->principal_point_y_px = cameraIntrinsics.principalPointY_;

    msg->deviceId = deviceId_;

    msg->descriptors_mat_data = vector<unsigned char>(descriptors.datastart,
                                                      descriptors.dataend);
    msg->descriptors_mat_cols = descriptors.cols;
    msg->descriptors_mat_rows = descriptors.rows;
    msg->descriptors_mat_type = descriptors.type();

    for (auto &kp:keyPoints)
    {
        KeyPoint kpRos;
        kpRos.x_pos = kp.pt.x;
        kpRos.y_pos = kp.pt.y;
        kpRos.angle = kp.angle;
        kpRos.class_id = kp.class_id;
        kpRos.octave = kp.octave;
        kpRos.response = kp.response;
        kpRos.size = kp.size;

        msg->keypoints.push_back(kpRos);
    }

    msg->image.header = msg->header;
    msg->image.format = "jpg";

    if (debugImage.total())
    {
        using namespace sensor_msgs;
        vector<unsigned char> jpgData;
        cv::imencode(".jpg", debugImage, jpgData);
        msg->image.data = jpgData;
    }

    publisher_.publish(msg);
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
