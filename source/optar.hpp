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
//#include <vector>

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
    } Settings;
    
    typedef struct _Point {
        int x, y;
    } Point;

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
    
    const std::map<std::string, double>& getStats() const;
    
private:
    class Impl;
    std::shared_ptr<OptarClient::Impl> pimpl_;
};

}

#endif
