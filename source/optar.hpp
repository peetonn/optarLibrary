//
// optar.hpp
//
//  Created by Peter Gusev on 5 November 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#ifndef __optar_hpp__
#define __optar_hpp__

#include "config.hpp"

namespace optar
{

const char* getLibraryVersion();

class OptarClient {
public:
    OptarClient();
    ~OptarClient();

    void processTexture(int w, int h, const void *rgbaData,
                        int &nKeypoints, int &nDescriptors);
};

}

#endif
