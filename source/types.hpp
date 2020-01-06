//
// types.hpp
//
//  Created by Peter Gusev on 18 December 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#ifndef __types_hpp__
#define __types_hpp__

#include <stdint.h>

namespace optar
{
    typedef struct _Vector2 {
        float x_, y_;
    } Vector2;
    typedef Vector2 Point2D;

    typedef struct _Vector3 {
        float x_,y_,z_;
    } Vector3;
    typedef Vector3 Point3D;

    typedef struct _Vector4 {
        float x_,y_,z_,w_;
    } Vector4;
    typedef Vector4 Quaternion;

    typedef struct _Pose {
        Vector3 position_;
        Vector4 rotation_;
    } Pose;
    typedef Pose Transform;

    typedef void(*OnWorldToArTransform)(const Transform&, int64_t tsUsec, void *userData);

    typedef struct _CameraIntrinsics {
        float focalLengthX_, focalLengthY_;
        int imageWidth_, imageHeight_;
        float principalPointX_, principalPointY_;
    } CameraIntrinsics;

    typedef struct _Settings {
        int rawImageScaleDownF_ = 1; // scale down factor for raw image
        int orbMaxPoints_ = 1000;
        int orbLevelsNumber_ = 10;
        double orbScaleFactor_ = 1.18f;
        double targetFps_ = 30;
        bool showDebugImage_ = false, sendDebugImage_ = false;
        const char* rosMasterUri_;
        const char* deviceId_;
        OnWorldToArTransform transformCallback_;
        void *userData_;

        const char* toString();
    } Settings;
}

#endif
