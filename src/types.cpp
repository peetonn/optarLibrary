//
// types.cpp
//
//  Created by Peter Gusev on 18 December 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#include <memory.h>
#include "types.hpp"
#include <sstream>

using namespace std;
using namespace optar;

const char*
_Settings::toString()
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
