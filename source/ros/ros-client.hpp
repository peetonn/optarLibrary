//
// ros-client.hpp
//
//  Created by Peter Gusev on 5 December 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#ifndef __ros_client_hpp__
#define __ros_client_hpp__

#include <memory>

namespace optar
{

/**
 * This class is a wrapper for ROS communication behind OPTAR.
 */
class RosClient {
public:
    static void initRos(const std::string &rosMasterUri);

private:

};

}

#endif
