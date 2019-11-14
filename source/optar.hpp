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

namespace optar
{

const char* getLibraryVersion();

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

    OptarClient(const Settings& settings);
    ~OptarClient();

    void processTexture(int w, int h, const void *rgbaData);
    
    const std::map<std::string, double>& getStats() const;
    
private:
    class Impl;
    std::shared_ptr<OptarClient::Impl> pimpl_;
};

}

#endif
