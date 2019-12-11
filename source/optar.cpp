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

#define DEBUG_SHOW(img)

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
    Impl(const OptarClient::Settings &settings)
    : settings_(settings)
    , orb_(ORB::create(settings_.orbMaxPoints_, settings_.orbScaleFactor_,
                       settings_.orbLevelsNumber_))
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
                        int &nKeypoints, OptarClient::Point **keypointsOut,
                        bool debugSaveImage);
    const map<string, double>& getStats() const
    {
        return stats_;
    }
    
private:
    OptarClient::Settings settings_;
    map<string, double> stats_;
    Ptr<Feature2D> orb_;

    // ROS components
    shared_ptr<ros_components::HeartbeatPublisher> heartbeatPublisher_;
    shared_ptr<ros_components::RosNtpClient> ntpClient_;

    void runOrb(const Mat &img,
                vector<KeyPoint> &keypoints, Mat &descriptors,
                bool debugSaveImage);
};

//******************************************************************************
const char*
OptarClient::_Settings::toString()
{
    stringstream ss;
    ss
    << "img scale down " << rawImageScaleDownF_
    << " ORB max points " << orbMaxPoints_
    << " ORB levels " << orbLevelsNumber_
    << " ORB scale " << orbScaleFactor_
    << " target FPS " << targetFps_
    << " show debug img " << showDebugImage_
    << " send debug img " << sendDebugImage_
    << " ROS master URI " << rosMasterUri_
    << " device ID " << deviceId_;

    static char buf[1024];
    memset(buf,0,1024);
    memcpy(buf, ss.str().c_str(), ss.str().size());

    return buf;
}

OptarClient::OptarClient(const Settings& settings)
: pimpl_(make_shared<OptarClient::Impl>(settings))
{
    OLOG_DEBUG("Optar Library Client. {}", getLibraryVersion());
}

OptarClient::~OptarClient(){}

void
OptarClient::processTexture(int w, int h, const void *rgbaData,
                            int &nKeypoints, OptarClient::Point **keypointsOut,
                            bool debugSaveImage)
{
    Mat imgWrapper = Mat(h,w, CV_8UC4, const_cast<void*>(rgbaData));
    pimpl_->processTexture(imgWrapper, nKeypoints, keypointsOut, debugSaveImage);
}

void
OptarClient::processTexture(int w, int h, int yStride,
                            const void *yuvData,
                            int &nKeyPoints,
                            Point **keyPoints,
                            bool debugSaveImage)
{
    Mat imgYV12 = Mat(h * 3/2, w, CV_8UC1, const_cast<void*>(yuvData));
    Mat imgDest;
//    cvtColor(imgYV12, imgDest, COLOR_YUV2GRAY_YV12);
//    cvtColor(imgYV12, imgDest, COLOR_YUV2GRAY_Y422);
    cvtColor(imgYV12, imgDest, COLOR_YUV2RGBA_I420 );
    pimpl_->processTexture(imgDest, nKeyPoints, keyPoints, debugSaveImage);
}

const map<string, double>&
OptarClient::getStats() const
{
    return pimpl_->getStats();
}

//******************************************************************************
void
OptarClient::Impl::processTexture(const Mat &img,
                                  int &nKeypoints, OptarClient::Point **keypointsOut,
                                  bool debugSaveImage)
{
    // - compute ORB descriptors
    vector<KeyPoint> keypoints;
    Mat descriptors;
    runOrb(img, keypoints, descriptors, debugSaveImage);

    nKeypoints = keypoints.size();
    
    if (keypoints.size())
    {
        if (keypointsOut)
        {
            *keypointsOut = (Point*)malloc(sizeof(Point)*keypoints.size());
            
            int i = 0;
            double s = settings_.rawImageScaleDownF_;
            for (auto kp : keypoints)
            {
                (*keypointsOut)[i].x = int(kp.pt.x*s);
                (*keypointsOut)[i++].y = int(kp.pt.y*s);
            }
        }
        
        // - obtain camera pose
        
        // - form an OPTAR-ROS message
        //   required data
        //      instance-time:
        //          - device ID <- ???
        //          - camera intrinsics <- GoogleARCore.Frame.CameraImage.ImageIntrinsics
        //      frame-time:
        //          - camera pose <- from PoseManager or current GoogleARCore.Frame.Pose
        //          - keypoints
        //          - descriptors
        //          - ARCore camera ROS frame ID <- ???
        //          - image time <- ROS time from NTP client
        
        // - send message
    }
    else
        OLOG_DEBUG("No image keypoints found. Skip frame");
}

void
OptarClient::Impl::runOrb(const Mat &imgWrapper,
                          vector<KeyPoint> &keypoints, Mat &descriptors,
                          bool debugSaveImage)
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
    if (settings_.showDebugImage_)
    {
        Mat kpImage;
        drawKeypoints(procImg, keypoints, kpImage);
        //DEBUG_SHOW(kpImage);
        if (debugSaveImage)
        {
            bool r = imwrite("/storage/emulated/0/Pictures/keypoints.jpg", kpImage);
            if (r)
            {
                OLOG_DEBUG("Saved debug image keypoints.jpg");
            }
            else
            {
                OLOG_DEBUG("Couldn't save debug image");
            }
        }
    }
#endif
    
    OLOG_DEBUG("ORB completed in {} (total {}). keypoints {}",
               stats_["orbCompute"], stats_["optarProc"], stats_["orbKp"]);
}

void
OptarClient::Impl::setupRosComponents()
{
    using namespace ros_components;
    RosClient::initRos(settings_.rosMasterUri_);
    heartbeatPublisher_ = RosClient::createHeartbeatPublisher(settings_.deviceId_);
    ntpClient_ = RosClient::createNtpClient();
}
