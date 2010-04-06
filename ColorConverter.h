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

#ifndef ANDROID_LIBCAMERA_COLOR_CONVERTER_H
#define ANDROID_LIBCAMERA_COLOR_CONVERTER_H

#include <utils/Errors.h>

namespace android {

void YUV420ToRGB565(int width, int height, void *src, void *dst);

void NV12ToRGB565(int width, int height, void *src, void *dst);

void NV12ToNV21(int width, int height, void *src, void *dst);

void NV12ToYV12(int width, int height, void *src, void *dst);

void YUYVToRGB8888(int width, int height, void *src, void *dst);

const char *cameraParametersFormat(int v4l2Format);
int V4L2Format(const char *cameraParamsFormat);

}; // namespace android

#endif // ANDROID_LIBCAMERA_COLOR_CONVERTER_H
