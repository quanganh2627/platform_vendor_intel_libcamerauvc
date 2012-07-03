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
#define LOG_TAG "Camera_PictureThread"

#include "PictureThread.h"
#include "LogHelper.h"
#include "Callbacks.h"
#include "ColorConverter.h"
#include <utils/Timers.h>

namespace android {

static const int MAX_EXIF_SIZE = 0xFFFF;
static const unsigned char JPEG_MARKER_SOI[2] = {0xFF, 0xD8}; // JPEG StartOfImage marker
static const unsigned char JPEG_MARKER_EOI[2] = {0xFF, 0xD9}; // JPEG EndOfImage marker

PictureThread::PictureThread() :
    Thread(true) // callbacks may call into java
    ,mMessageQueue("PictureThread", MESSAGE_ID_MAX)
    ,mThreadRunning(false)
    ,mCallbacks(Callbacks::getInstance())
{
    LOG1("@%s", __FUNCTION__);
    memset(&mOutBuf, 0, sizeof(mOutBuf));
    memset(&mExifBuf, 0, sizeof(mExifBuf));
}

PictureThread::~PictureThread()
{
    LOG1("@%s", __FUNCTION__);
    if (mOutBuf.buff != NULL) {
        mOutBuf.buff->release(mOutBuf.buff);
    }
    if (mExifBuf.buff != NULL) {
        mExifBuf.buff->release(mExifBuf.buff);
    }
}

/*
 * encodeToJpeg: encodes the given buffer and creates the final JPEG file
 * Input:  mainBuf  - buffer containing the main picture image
 *         thumbBuf - buffer containing the thumbnail image (optional, can be NULL)
 * Output: destBuf  - buffer containing the final JPEG image including EXIF header
 *         Note that, if present, thumbBuf will be included in EXIF header
 */
status_t PictureThread::encodeToJpeg(CameraBuffer *mainBuf, CameraBuffer *thumbBuf, CameraBuffer *destBuf)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    JpegCompressor::InputBuffer inBuf;
    JpegCompressor::OutputBuffer outBuf;
    nsecs_t startTime = systemTime();
    nsecs_t endTime;

    if (mOutBuf.buff == NULL || mOutBuf.buff->data == NULL || mOutBuf.buff->size <= 0) {
        int bufferSize = (mConfig.picture.width * mConfig.picture.height * 2);
        mCallbacks->allocateMemory(&mOutBuf, bufferSize);
    }
    if (mExifBuf.buff == NULL || mExifBuf.buff->data == NULL || mExifBuf.buff->size <= 0) {
        mCallbacks->allocateMemory(&mExifBuf, MAX_EXIF_SIZE);
    }
    if (mOutBuf.buff == NULL || mOutBuf.buff->data == NULL) {
        LOGE("Could not allocate memory for temp buffer!");
        return NO_MEMORY;
    }
    LOG1("Out buffer: @%p (%d bytes)", mOutBuf.buff->data, mOutBuf.buff->size);
    LOG1("Exif buffer: @%p (%d bytes)", mExifBuf.buff->data, mExifBuf.buff->size);
    // Convert and encode the thumbnail, if present and EXIF maker is initialized

    if (mConfig.exif.enableThumb) {

        LOG1("Encoding thumbnail");

        // setup the JpegCompressor input and output buffers
        inBuf.clear();
        inBuf.buf = (unsigned char*)thumbBuf->buff->data;
        inBuf.width = mConfig.thumbnail.width;
        inBuf.height = mConfig.thumbnail.height;
        inBuf.format = mConfig.thumbnail.format;
        inBuf.size = frameSize(mConfig.thumbnail.format,
                mConfig.thumbnail.width,
                mConfig.thumbnail.height);
        outBuf.clear();
        outBuf.buf = (unsigned char*)mOutBuf.buff->data;
        outBuf.width = mConfig.thumbnail.width;
        outBuf.height = mConfig.thumbnail.height;
        outBuf.quality = mConfig.thumbnail.quality;
        outBuf.size = mOutBuf.buff->size;
        endTime = systemTime();
        int size = compressor.encode(inBuf, outBuf);
        LOG1("Thumbnail JPEG size: %d (time to encode: %ums)", size, (unsigned)((systemTime() - endTime) / 1000000));
        if (size > 0) {
            encoder.setThumbData(outBuf.buf, size);
        } else {
            // This is not critical, we can continue with main picture image
            LOGE("Could not encode thumbnail stream!");
        }
    } else {
        LOG1("Skipping thumbnail");
    }
    int totalSize = 0;
    unsigned int exifSize = 0;
    // Copy the SOI marker
    unsigned char* currentPtr = (unsigned char*)mExifBuf.buff->data;
    memcpy(currentPtr, JPEG_MARKER_SOI, sizeof(JPEG_MARKER_SOI));
    totalSize += sizeof(JPEG_MARKER_SOI);
    currentPtr += sizeof(JPEG_MARKER_SOI);
    if (encoder.makeExif(currentPtr, &mConfig.exif, &exifSize, false) != JPG_SUCCESS)
        LOGE("Error making EXIF");
    currentPtr += exifSize;
    totalSize += exifSize;
    // Copy the EOI marker
    memcpy(currentPtr, (void*)JPEG_MARKER_EOI, sizeof(JPEG_MARKER_EOI));
    totalSize += sizeof(JPEG_MARKER_EOI);
    currentPtr += sizeof(JPEG_MARKER_EOI);
    exifSize = totalSize;

    // Convert and encode the main picture image
    // setup the JpegCompressor input and output buffers
    inBuf.clear();
    inBuf.buf = (unsigned char *) mainBuf->buff->data;

    inBuf.width = mConfig.picture.width;
    inBuf.height = mConfig.picture.height;
    inBuf.format = mConfig.picture.format;
    inBuf.size = frameSize(mConfig.picture.format,
            mConfig.picture.width,
            mConfig.picture.height);
    outBuf.clear();
    outBuf.buf = (unsigned char*)mOutBuf.buff->data;
    outBuf.width = mConfig.picture.width;
    outBuf.height = mConfig.picture.height;
    outBuf.quality = mConfig.picture.quality;
    outBuf.size = mOutBuf.buff->size;
    endTime = systemTime();
    int mainSize = compressor.encode(inBuf, outBuf);
    LOG1("Picture JPEG size: %d (time to encode: %ums)", mainSize, (unsigned)((systemTime() - endTime) / 1000000));
    if (mainSize > 0) {
        // We will skip SOI marker from final file
        totalSize += (mainSize - sizeof(JPEG_MARKER_SOI));
    } else {
        LOGE("Could not encode picture stream!");
        status = UNKNOWN_ERROR;
    }

    if (status == NO_ERROR) {
        mCallbacks->allocateMemory(destBuf, totalSize);
        if (destBuf->buff == NULL) {
            LOGE("No memory for final JPEG file!");
            status = NO_MEMORY;
        }
    }
    if (status == NO_ERROR) {
        // Copy EXIF (it will also have the SOI and EOI markers
        memcpy(destBuf->buff->data, mExifBuf.buff->data, exifSize);
        // Copy the final JPEG stream into the final destination buffer, but exclude the SOI marker
        char *copyTo = (char*)destBuf->buff->data + exifSize;
        char *copyFrom = (char*)mOutBuf.buff->data + sizeof(JPEG_MARKER_SOI);
        memcpy(copyTo, copyFrom, mainSize - sizeof(JPEG_MARKER_SOI));
    }
    LOG1("Total JPEG size: %d (time to encode: %ums)", totalSize, (unsigned)((systemTime() - startTime) / 1000000));
    return status;
}


status_t PictureThread::encode(CameraBuffer *snaphotBuf, CameraBuffer *postviewBuf)
{
    LOG1("@%s", __FUNCTION__);
    Message msg;
    msg.id = MESSAGE_ID_ENCODE;
    msg.data.encode.snaphotBuf = *snaphotBuf;
    if (postviewBuf) {
        msg.data.encode.postviewBuf = *postviewBuf;
    } else {
        // thumbnail is optional
        msg.data.encode.postviewBuf.buff = NULL;
    }
    return mMessageQueue.send(&msg);
}

void PictureThread::getDefaultParameters(CameraParameters *params)
{
    LOG1("@%s", __FUNCTION__);
    if (!params) {
        LOGE("null params");
        return;
    }

    params->setPictureFormat(CameraParameters::PIXEL_FORMAT_JPEG);
    params->set(CameraParameters::KEY_SUPPORTED_PICTURE_FORMATS,
            CameraParameters::PIXEL_FORMAT_JPEG);
    params->set(CameraParameters::KEY_JPEG_QUALITY, "80");
    params->set(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY, "50");
}

void PictureThread::setConfig(Config *config)
{
    mConfig = *config;
}

status_t PictureThread::flushBuffers()
{
    LOG1("@%s", __FUNCTION__);
    Message msg;
    msg.id = MESSAGE_ID_FLUSH;
    mMessageQueue.remove(MESSAGE_ID_ENCODE);
    return mMessageQueue.send(&msg, MESSAGE_ID_FLUSH);
}

status_t PictureThread::handleMessageExit()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    mThreadRunning = false;
    return status;
}

status_t PictureThread::handleMessageEncode(MessageEncode *msg)
{
    LOG1("@%s: snapshot ID = %d", __FUNCTION__, msg->snaphotBuf.id);
    status_t status = NO_ERROR;
    int exifSize = 0;
    int totalSize = 0;
    CameraBuffer jpegBuf;

    if (mConfig.picture.width == 0 ||
        mConfig.picture.height == 0 ||
        mConfig.picture.format == 0) {
        LOGE("Picture information not set yet!");
        return UNKNOWN_ERROR;
    }

    // Encode the image
    CameraBuffer *postviewBuf = msg->postviewBuf.buff == NULL ? NULL : &msg->postviewBuf;
    if ((status = encodeToJpeg(&msg->snaphotBuf, postviewBuf, &jpegBuf)) == NO_ERROR) {
        mCallbacks->compressedFrameDone(&jpegBuf);
    } else {
        LOGE("Error generating JPEG image!");
        if (jpegBuf.buff != NULL && jpegBuf.buff->data != NULL) {
            LOG1("Releasing jpegBuf @%p", jpegBuf.buff->data);
            jpegBuf.buff->release(jpegBuf.buff);
        }
    }

    // When the encoding is done, send back the buffers to camera

    IBufferOwner *owner = msg->snaphotBuf.owner;
    if (owner) {
        owner->returnBuffer(&msg->snaphotBuf, postviewBuf);
    }

    return status;
}

status_t PictureThread::handleMessageFlush()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    mMessageQueue.reply(MESSAGE_ID_FLUSH, status);
    return status;
}

status_t PictureThread::waitForAndExecuteMessage()
{
    LOG2("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    Message msg;
    mMessageQueue.receive(&msg);

    switch (msg.id) {

        case MESSAGE_ID_EXIT:
            status = handleMessageExit();
            break;

        case MESSAGE_ID_ENCODE:
            status = handleMessageEncode(&msg.data.encode);
            break;

        case MESSAGE_ID_FLUSH:
            status = handleMessageFlush();
            break;

        default:
            status = BAD_VALUE;
            break;
    };
    return status;
}

bool PictureThread::threadLoop()
{
    LOG2("@%s", __FUNCTION__);
    status_t status = NO_ERROR;

    mThreadRunning = true;
    while (mThreadRunning)
        status = waitForAndExecuteMessage();

    return false;
}

status_t PictureThread::requestExitAndWait()
{
    LOG1("@%s", __FUNCTION__);
    Message msg;
    msg.id = MESSAGE_ID_EXIT;

    // tell thread to exit
    // send message asynchronously
    mMessageQueue.send(&msg);

    // propagate call to base class
    return Thread::requestExitAndWait();
}

} // namespace android
