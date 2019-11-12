//
// optar.cpp
//
//  Created by Peter Gusev on 5 November 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#include "optar.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#define MAX_FEATURES 500 // ???

using namespace std;
using namespace cv;
using namespace optar;

namespace optar {

const char* getLibraryVersion()
{
    char msg[256];
    sprintf(msg, "optar v0.0.1 (opencv %s)", CV_VERSION);
    return msg;
}

}
//******************************************************************************
OptarClient::OptarClient()
{
    
}

OptarClient::~OptarClient()
{
    
}

void
OptarClient::processTexture(int w, int h, const void *rgbaData,
                            int &nKeypoints, int &nDescriptors)
{
    Mat imgWrapper = Mat(h,w, CV_8UC4, const_cast<void*>(rgbaData));
    Mat imgGray;
    
    cvtColor(imgWrapper, imgGray, COLOR_RGBA2GRAY);
    
    vector<KeyPoint> keypoints;
    Mat descriptors;
     
    // Detect ORB features and compute descriptors.
    Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
    orb->detectAndCompute(imgGray, Mat(), keypoints, descriptors);
    
    nKeypoints = (int)keypoints.size();
//    nDescriptors = ()descriptors.size();
}

//}
