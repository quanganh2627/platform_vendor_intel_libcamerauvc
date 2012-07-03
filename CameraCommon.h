/*
 * Copyright (C) 2011 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ANDROID_LIBCAMERA_COMMON_H
#define ANDROID_LIBCAMERA_COMMON_H

#include <camera.h>
#include <linux/videodev2.h>
#include <stdio.h>



//This file define the general configuration for the camera driver

#define BPP 2 // bytes per pixel
#define MAX_PARAM_VALUE_LENGTH 32
#define MAX_BURST_BUFFERS 32

namespace android {

struct CameraBuffer;

class IBufferOwner
{
public:
    virtual void returnBuffer(CameraBuffer* buff1, CameraBuffer* buff2 = NULL) = 0;
    virtual ~IBufferOwner(){};
};

enum BufferType {
    BUFFER_TYPE_PREVIEW,
    BUFFER_TYPE_VIDEO,
    BUFFER_TYPE_SNAPSHOT,
    BUFFER_TYPE_THUMBNAIL
};

struct CameraBuffer {
    camera_memory_t *buff;
    int id;                 // id for debugging data flow path
    int driverPrivate;      // Private to the CameraDriver class.
                            // No other classes should touch this
    BufferType type;
    IBufferOwner* owner;    //owner who is responsible to enqueue back to CameraDriver
    int format;
};

struct CameraWindow {
    int x_left;
    int x_right;
    int y_top;
    int y_bottom;
    int weight;
};

static int frameSize(int format, int width, int height)
{
    int size = 0;
    switch (format) {
        case V4L2_PIX_FMT_YUV420:
        case V4L2_PIX_FMT_YVU420:
        case V4L2_PIX_FMT_NV12:
        case V4L2_PIX_FMT_NV21:
        case V4L2_PIX_FMT_YUV411P:
        case V4L2_PIX_FMT_YUV422P:
            size = (width * height * 3 / 2);
            break;
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_Y41P:
        case V4L2_PIX_FMT_UYVY:
            size = (width * height *  2);
            break;
        case V4L2_PIX_FMT_RGB565:
            size = (width * height * BPP);
            break;
        default:
            size = (width * height * 2);
    }

    return size;
}

static int paddingWidth(int format, int width, int height)
{
    int padding = 0;
    switch (format) {
    //64bit align for 1.5byte per pixel
    case V4L2_PIX_FMT_YUV420:
    case V4L2_PIX_FMT_YVU420:
    case V4L2_PIX_FMT_NV12:
    case V4L2_PIX_FMT_NV21:
    case V4L2_PIX_FMT_YUV411P:
    case V4L2_PIX_FMT_YUV422P:
        padding = (width + 63) / 64 * 64;
        break;
    //32bit align for 2byte per pixel
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_Y41P:
    case V4L2_PIX_FMT_UYVY:
        padding = width;
        break;
    case V4L2_PIX_FMT_RGB565:
        padding = (width + 31) / 32 * 32;
        break;
    default:
        padding = (width + 63) / 64 * 64;
    }
    return padding;
}

static const char* v4l2Fmt2Str(int format)
{
    static char fourccBuf[5];
    memset(&fourccBuf[0], 0, sizeof(fourccBuf));
    char *fourccPtr = (char*) &format;
    snprintf(fourccBuf, sizeof(fourccBuf), "%c%c%c%c", *fourccPtr, *(fourccPtr+1), *(fourccPtr+2), *(fourccPtr+3));
    return &fourccBuf[0];
}

}
#endif // ANDROID_LIBCAMERA_COMMON_H
