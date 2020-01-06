//
// optar.cpp
//
//  Created by Peter Gusev on 5 November 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#include "optar.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <chrono>

#include "config.hpp"
#include "logging.hpp"
#include "ros/ros-client.hpp"
#include "utils/clock.hpp"

#define DEBUG_SHOW(img)
#define DEBUG_IMG_WIDTH 200.

#if DEBUG

#ifndef __ANDROID__
#undef DEBUG_SHOW
#define DEBUG_SHOW(img) cv::imshow("optar-"#img, img); cv::waitKey(1);
#endif

#endif

using namespace std;
using namespace std::chrono;
using namespace cv;
using namespace optar;

void __attribute__((constructor)) lib_ctor(){}
void __attribute__((destructor)) lib_dtor(){}

namespace optar {

namespace profiling // NOTE: not thread-safe!
{

using namespace std::chrono;
static high_resolution_clock::time_point t, snapT;

inline void start() { t = high_resolution_clock::now(); snapT = t; }

template<class CastT>
inline int snap() {
    auto now = high_resolution_clock::now();
    auto d = duration_cast<CastT>(now - snapT).count();
    snapT = now;
    return (int)d;
}

template<class CastT>
inline int total() { return (int)duration_cast<CastT>(high_resolution_clock::now() - t).count(); }

}

const char* getLibraryVersion()
{
    static char msg[256];
#if DEBUG
    sprintf(msg, "optar v%s-%s (opencv %s)", PACKAGE_VERSION, "debug", CV_VERSION);
#else
    sprintf(msg, "optar v%s-%s (opencv %s)", PACKAGE_VERSION, "release", CV_VERSION);
#endif
    return msg;
}

void registerLogCallback(LogCallback clbck)
{
    registerCallback(getLogger("optar"), [clbck](const string& msg){
        clbck(msg.c_str());
    });
}

}

//******************************************************************************
class OptarClient::Impl : public enable_shared_from_this<OptarClient::Impl> {
public:
    Impl(const Settings &settings)
    : settings_(settings)
    , orb_(ORB::create(settings_.orbMaxPoints_, settings_.orbScaleFactor_,
                       settings_.orbLevelsNumber_))
    , rateThrottle_(1) //60)
    , lastRunTsMs_(0)
    {
        setupRosComponents();
    }

    ~Impl()
    {
#ifndef __ANDROID__
#if DEBUG
        destroyAllWindows();
#endif
#endif
    }

    void processTexture(const Mat &img,
                        int &nKeypoints,
                        CameraIntrinsics cameraIntrinsics,
                        Point2D **keypointsOut,
                        bool debugSaveImage);
    void processArPose(const Pose &pose);

    const map<string, double>& getStats() const
    {
        return stats_;
    }

    Transform getLastTransform() const;

private:
    Settings settings_;
    map<string, double> stats_;
    Ptr<Feature2D> orb_;
    double rateThrottle_;
    int64_t lastRunTsMs_;

    // ROS components
    shared_ptr<ros_components::HeartbeatPublisher> heartbeatPublisher_;
    shared_ptr<ros_components::RosNtpClient> ntpClient_;
    shared_ptr<ros_components::ArPosePublisher> arPosePublisher_;
    shared_ptr<ros_components::OptarCameraPublisher> featuresPublisher_;
    shared_ptr<ros_components::TfListener> tfListener_;

    void runOrb(const Mat &img,
                vector<KeyPoint> &keypoints, Mat &descriptors,
                bool debugSaveImage,
                Mat &debugImage);
    void setupRosComponents();
    // called by TfListener whenever new transform (world -> AR) was retrieved
    void onNewTransform(const Transform& t, int64_t tsUsec);
};

//******************************************************************************
OptarClient::OptarClient(const Settings& settings)
: pimpl_(make_shared<OptarClient::Impl>(settings))
{
    OLOG_DEBUG("Optar Library Client. {}", getLibraryVersion());
}

OptarClient::~OptarClient(){}

void
OptarClient::processTexture(int w, int h, const void *rgbaData,
                            int &nKeypoints,
                            CameraIntrinsics cameraIntrinsics,
                            bool debugSaveImage,
                            Point2D **keypointsOut)
{
    Mat imgWrapper = Mat(h,w, CV_8UC4, const_cast<void*>(rgbaData));
    pimpl_->processTexture(imgWrapper, nKeypoints, cameraIntrinsics, keypointsOut, debugSaveImage);
}

void
OptarClient::processTexture(int w, int h, int yStride,
                            const void *yuvData,
                            int &nKeyPoints,
                            CameraIntrinsics cameraIntrinsics,
                            bool debugSaveImage,
                            Point2D **keyPoints)
{
    Mat imgYV12 = Mat(h * 3/2, w, CV_8UC1, const_cast<void*>(yuvData));
    Mat imgDest;
//    cvtColor(imgYV12, imgDest, COLOR_YUV2GRAY_YV12);
//    cvtColor(imgYV12, imgDest, COLOR_YUV2GRAY_Y422);
    cvtColor(imgYV12, imgDest, COLOR_YUV2RGBA_I420 );
    pimpl_->processTexture(imgDest, nKeyPoints, cameraIntrinsics, keyPoints, debugSaveImage);
}

void
OptarClient::processArPose(const Pose &pose)
{
    pimpl_->processArPose(pose);
}

Transform
OptarClient::getLastTransform() const
{
    return pimpl_->getLastTransform();
}

const map<string, double>&
OptarClient::getStats() const
{
    return pimpl_->getStats();
}

//******************************************************************************
void
OptarClient::Impl::processTexture(const Mat &img,
                                  int &nKeypoints,
                                  CameraIntrinsics cameraIntrinsics,
                                  Point2D **keypointsOut,
                                  bool debugSaveImage)
{
    // throttle processing
    int64_t now = clock::millisecondTimestamp();

    if (now - lastRunTsMs_ < 1000./rateThrottle_)
        return;

    lastRunTsMs_ = now;

    // - compute ORB descriptors
    vector<KeyPoint> keypoints;
    Mat descriptors;
    Mat debugImage;
    runOrb(img, keypoints, descriptors, debugSaveImage, debugImage);

    nKeypoints = keypoints.size();

    if (keypoints.size())
    {
        if (keypointsOut)
        {
            *keypointsOut = (Point2D*)malloc(sizeof(Point2D)*keypoints.size());

            int i = 0;
            double s = settings_.rawImageScaleDownF_;
            for (auto kp : keypoints)
            {
                (*keypointsOut)[i].x_ = (kp.pt.x*s);
                (*keypointsOut)[i++].y_ = (kp.pt.y*s);
            }
        }

        cameraIntrinsics.imageWidth_ /= settings_.rawImageScaleDownF_;
        cameraIntrinsics.imageHeight_ /= settings_.rawImageScaleDownF_;
        featuresPublisher_->publish(ntpClient_->getSyncTime(),
                                    arPosePublisher_->getLastPose(),
                                    cameraIntrinsics,
                                    descriptors,
                                    keypoints,
                                    debugImage);
    }
    else
        OLOG_DEBUG("No image keypoints found. Skip frame");
}

void
OptarClient::Impl::runOrb(const Mat &imgWrapper,
                          vector<KeyPoint> &keypoints, Mat &descriptors,
                          bool debugSaveImage,
                          Mat &debugImage)
{
    profiling::start();

    Mat procImg;

    double resizeF = 1./(double)settings_.rawImageScaleDownF_;
    resize(imgWrapper, procImg, Size(), resizeF, resizeF, INTER_AREA);

    stats_["resize"] = profiling::snap<chrono::microseconds>();

    cvtColor(procImg, procImg, COLOR_RGBA2GRAY);
    stats_["cvtColor"] = profiling::snap<chrono::microseconds>();

    // Detect ORB features and compute descriptors.
    orb_->detectAndCompute(procImg, Mat(), keypoints, descriptors);

    stats_["orbCompute"] = profiling::snap<chrono::microseconds>();
    stats_["orbKp"] = keypoints.size();
    stats_["optarProc"] = profiling::total<chrono::microseconds>();

    if (settings_.showDebugImage_)
    {
        //DEBUG_SHOW(procImg);
    }

#if DEBUG
    if (debugSaveImage)
    {
        Mat kpImage;
        drawKeypoints(procImg, keypoints, kpImage);

        if (kpImage.cols)
        {
            debugImage = Mat(Size(DEBUG_IMG_WIDTH ,
                                  DEBUG_IMG_WIDTH / (float)kpImage.cols * (float)kpImage.rows),
                             kpImage.type());

            resize(kpImage, debugImage, debugImage.size(), 0, 0);
        }
        //DEBUG_SHOW(kpImage);
    }
#endif

    OLOG_DEBUG("ORB completed in {} (total {}). keypoints {}",
               stats_["orbCompute"], stats_["optarProc"], stats_["orbKp"]);
}

void
OptarClient::Impl::processArPose(const Pose &pose)
{
    arPosePublisher_->publishPose(pose);
}

void
OptarClient::Impl::setupRosComponents()
{
    using namespace ros_components;
    RosClient::initRos(settings_.rosMasterUri_);
    heartbeatPublisher_ = RosClient::createHeartbeatPublisher(settings_.deviceId_);
    ntpClient_ = RosClient::createNtpClient();
    arPosePublisher_ = RosClient::createPosePublisher(settings_.deviceId_, 30);
    featuresPublisher_ = RosClient::createOptarPublisher(settings_.deviceId_);
    tfListener_ = RosClient::createTfListener(settings_.deviceId_, bind(&OptarClient::Impl::onNewTransform, this, _1, _2));
}

void
OptarClient::Impl::onNewTransform(const Transform& t, int64_t tsUsec)
{
    if (settings_.transformCallback_)
    {
        settings_.transformCallback_(t, tsUsec, settings_.userData_);
    }
}

Transform
OptarClient::Impl::getLastTransform() const
{
    Transform t;
    memset(&t, 0, sizeof(t));

    if (tfListener_)
        t = tfListener_->getLastTransform();

    return t;
}
