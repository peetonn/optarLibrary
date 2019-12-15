//
// optar.hpp
//
//  Created by Peter Gusev on 5 November 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#ifndef __optar_hpp__
#define __optar_hpp__

#include "config.hpp"

#include <map>
#include <memory>

namespace optar
{

typedef void(*LogCallback)(const char*);

const char* getLibraryVersion();
void registerLogCallback(LogCallback clbck);

class OptarClient {
public:
    typedef struct _Settings {
        int rawImageScaleDownF_ = 2; // scale down factor for raw image
        int orbMaxPoints_ = 1000;
        int orbLevelsNumber_ = 10;
        double orbScaleFactor_ = 1.18f;
        double targetFps_ = 30;
        bool showDebugImage_ = false, sendDebugImage_ = false;
        const char* rosMasterUri_;
        const char* deviceId_;

        const char* toString();
    } Settings;

    typedef struct _Point {
        int x, y;
    } Point;

    typedef struct _Vector3 {
        float x_,y_,z_;
    } Vector3;

    typedef struct _Vector4 {
        float x_,y_,z_,w_;
    } Vector4;

    OptarClient(const Settings& settings);
    ~OptarClient();

    /**
     * Runs OPTAR algorithms on the texture and sends results to ROS server.
     * @param w Width of the image (texture)
     * @param h Heioght of the image
     * @param rgbaData RGBA pixel data of the image (texture)
     * @param nKeyPoints Returns number of keypoints detected
     * @param keyPoints If not null, this function will store keypoint data in this array. Client code is responsible for calling free().
     */
    void processTexture(int w, int h, const void *rgbaData,
                        int &nKeyPoints,
                        Point **keyPoints = nullptr,
                        bool debugSaveImage = false);

    /**
     * Runs OPTAR algorithms on the texture and sends results to ROS server.
     * @param w Width of the image (texture)
     * @param h Height of the image
     * @param yStride Y row stride
     * @param yuvData YUV plane data
     * @param nKeyPoints Returns number of keypoints detected
     * @param keyPoints If not null, this function will store keypoint data in this array. Client code is responsible for calling free().
     */
    void processTexture(int w, int h, int yStride,
                        const void *yuvData,
                        int &nKeyPoints,
                        Point **keyPoints = nullptr,
                        bool debugSaveImage = false);

    /**
     * Processes AR pose and pubblishes it in a ROS topic.
     * This also monitors pose and detects changes to it made by AR framework.
     * @param position Position of the pose represented using a point in 3D space
     * @param rotation Rotation of the pose represented using a quaternion
     */
    void processArPose(const Vector3 &position, const Vector4 &rotation);

    const std::map<std::string, double>& getStats() const;

private:
    class Impl;
    std::shared_ptr<OptarClient::Impl> pimpl_;
};

}

#endif
