//
// helpers.cpp
//
//  Created by Peter Gusev on 16 October 2021.
//

#include "helpers.hpp"

#include <stdlib.h>
#include "logging.hpp"

#if __APPLE__
#include "objc/ros-ios.hpp"
#endif

using namespace std;

string optar::helpers::getCrossPlatformWriteableFolder()
{
#if __ANDROID__
    char cmdLine[512];
    FILE *f = fopen("/proc/self/cmdline", "r");
    if (f)
    {
        size_t c = fread(cmdLine, 512, 1, f);
    }
    else
        DLOG_ERROR("FAILED TO READ /proc/self/cmdline");
    
    return "/data/data/"+string(cmdLine);
#elif __APPLE__
    return optar::helpers::ios::getDeviceDocumentsDirectory()+"/optar";
#else
    return "";
#endif
}

