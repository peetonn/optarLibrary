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

#define MAX_FEATURES 500 // ???

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
    registerCallback(getLogger("optar"),  [clbck](const string& msg){
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
    {}
    ~Impl()
    {
#ifndef __ANDROID__
#if DEBUG
        destroyAllWindows();
#endif
#endif
    }
    
    void processTexture(int w, int h, const void *rgbaData,
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
    
    void runOrb(int w, int h, const void *rgbaData,
                vector<KeyPoint> &keypoints, Mat &descriptors,
                bool debugSaveImage = false);
};

//******************************************************************************
OptarClient::OptarClient(const Settings& settings)
: pimpl_(make_shared<OptarClient::Impl>(settings))
{
}

OptarClient::~OptarClient(){}

void
OptarClient::processTexture(int w, int h, const void *rgbaData,
                            int &nKeypoints, OptarClient::Point **keypointsOut,
                            bool debugSaveImage)
{
    pimpl_->processTexture(w, h, rgbaData, nKeypoints, keypointsOut,
                           debugSaveImage);
}

const map<string, double>&
OptarClient::getStats() const
{
    return pimpl_->getStats();
}

//******************************************************************************
void
OptarClient::Impl::processTexture(int w, int h, const void *rgbaData,
                                  int &nKeypoints, OptarClient::Point **keypointsOut,
                                  bool debugSaveImage)
{
    // - compute ORB descriptors
    vector<KeyPoint> keypoints;
    Mat descriptors;
    runOrb(w, h, rgbaData, keypoints, descriptors);
    
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
OptarClient::Impl::runOrb(int w, int h, const void *rgbaData,
                          vector<KeyPoint> &keypoints, Mat &descriptors,
                          bool debugSaveImage)
{
    profiling::start();
    
    Mat imgWrapper = Mat(h,w, CV_8UC4, const_cast<void*>(rgbaData));
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
        DEBUG_SHOW(procImg);
    }
    
#if DEBUG
    if (settings_.showDebugImage_)
    {
        Mat kpImage;
        drawKeypoints(procImg, keypoints, kpImage);
        DEBUG_SHOW(kpImage);
        if (debugSaveImage)
            imwrite("keypoints.jpg", kpImage);
    }
#endif
    
    OLOG_DEBUG("ORB completed in {} (total {}). keypoints {}",
               stats_["orbCompute"], stats_["optarProc"], stats_["orbKp"]);
}
