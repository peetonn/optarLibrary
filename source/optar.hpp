//
// optar.hpp
//
//  Created by Peter Gusev on 5 November 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#ifndef __optar_hpp__
#define __optar_hpp__

#include "config.hpp"
#include "types.hpp"

#include <map>
#include <memory>

namespace optar
{

typedef void(*LogCallback)(const char*);

const char* getLibraryVersion();
void registerLogCallback(LogCallback clbck);

class OptarClient {
public:
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
                        CameraIntrinsics cameraIntrinsics,
                        bool debugSaveImage = false,
                        Point2D **keyPoints = nullptr);

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
                        CameraIntrinsics cameraIntrinsics,
                        bool debugSaveImage = false,
                        Point2D **keyPoints = nullptr);

    /**
     * Processes AR pose and pubblishes it in a ROS topic.
     * This also monitors pose and detects changes to it made by AR framework.
     * @param position Position of the pose represented using a point in 3D space
     * @param rotation Rotation of the pose represented using a quaternion
     */
    void processArPose(const Pose& pose);

    /**
     * Returns last received transform (world -> AR tracking space) if available.
     */
    Transform getLastTransform() const;

    /**
     * Returns statistics dictionary.
     */
    const std::map<std::string, double>& getStats() const;

private:
    class Impl;
    std::shared_ptr<OptarClient::Impl> pimpl_;
};

}

#endif
