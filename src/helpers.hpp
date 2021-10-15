//
// helpers.hpp
//
//  Created by Peter Gusev on 16 October 2021.
//

#ifndef __helpers_hpp__
#define __helpers_hpp__

#include <string>

namespace optar
{
namespace helpers
{
    // Returns cross-platform writeable folder for the current device.
    std::string getCrossPlatformWriteableFolder();

}
}

#endif
