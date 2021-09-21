//
//  ros-client.m
//  optarLibrary
//
//  Created by Peter Gusev on 8/23/21.
//

#include "ros-ios.hpp"
#include <TargetConditionals.h>
#import <Foundation/Foundation.h>

std::string optar::ros_components::ios::getDeviceDocumentsDirectory()
{
    NSArray *paths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    NSString *documentsDirectory = [paths objectAtIndex:0];

    return [documentsDirectory cStringUsingEncoding:NSASCIIStringEncoding];
}
