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
#define LOG_TAG "Camera_Driver"

#include <Exif.h>
#include "LogHelper.h"
#include "CameraDriver.h"
#include "Callbacks.h"
#include "ColorConverter.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <cutils/properties.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))

#define DEFAULT_SENSOR_FPS      15.0

#define RESOLUTION_14MP_WIDTH   4352
#define RESOLUTION_14MP_HEIGHT  3264
#define RESOLUTION_8MP_WIDTH    3264
#define RESOLUTION_8MP_HEIGHT   2448
#define RESOLUTION_5MP_WIDTH    2560
#define RESOLUTION_5MP_HEIGHT   1920
#define RESOLUTION_1080P_WIDTH  1920
#define RESOLUTION_1080P_HEIGHT 1080
#define RESOLUTION_720P_WIDTH   1280
#define RESOLUTION_720P_HEIGHT  720
#define RESOLUTION_480P_WIDTH   768
#define RESOLUTION_480P_HEIGHT  480
#define RESOLUTION_VGA_WIDTH    640
#define RESOLUTION_VGA_HEIGHT   480
#define RESOLUTION_POSTVIEW_WIDTH    320
#define RESOLUTION_POSTVIEW_HEIGHT   240

// TODO: revisit this legacy implementation
#define MAX_BACK_CAMERA_PREVIEW_WIDTH           RESOLUTION_1080P_WIDTH
#define MAX_BACK_CAMERA_PREVIEW_HEIGHT          RESOLUTION_1080P_HEIGHT
#define MAX_BACK_CAMERA_SNAPSHOT_WIDTH          RESOLUTION_1080P_WIDTH
#define MAX_BACK_CAMERA_SNAPSHOT_HEIGHT         RESOLUTION_1080P_HEIGHT
#define MAX_BACK_CAMERA_VIDEO_WIDTH             RESOLUTION_1080P_WIDTH
#define MAX_BACK_CAMERA_VIDEO_HEIGHT            RESOLUTION_1080P_HEIGHT

#define MAX_FRONT_CAMERA_PREVIEW_WIDTH          RESOLUTION_1080P_WIDTH
#define MAX_FRONT_CAMERA_PREVIEW_HEIGHT         RESOLUTION_1080P_HEIGHT
#define MAX_FRONT_CAMERA_SNAPSHOT_WIDTH         RESOLUTION_1080P_WIDTH
#define MAX_FRONT_CAMERA_SNAPSHOT_HEIGHT        RESOLUTION_1080P_HEIGHT
#define MAX_FRONT_CAMERA_VIDEO_WIDTH            RESOLUTION_1080P_WIDTH
#define MAX_FRONT_CAMERA_VIDEO_HEIGHT           RESOLUTION_1080P_HEIGHT

namespace android {

////////////////////////////////////////////////////////////////////
//                          STATIC DATA
////////////////////////////////////////////////////////////////////

CameraDriver::CameraSensor *CameraDriver::mCameraSensor[MAX_CAMERAS];
Mutex CameraDriver::mCameraSensorLock;
int CameraDriver::numCameras = 0;

////////////////////////////////////////////////////////////////////
//                          PUBLIC METHODS
////////////////////////////////////////////////////////////////////

CameraDriver::CameraDriver(int cameraId) :
    mMode(MODE_NONE)
    ,mCallbacks(Callbacks::getInstance())
    ,mNumBuffers(NUM_DEFAULT_BUFFERS)
    ,mPreviewBuffers(NULL)
    ,mRecordingBuffers(NULL)
    ,mNumPreviewBuffersQueued(0)
    ,mNumRecordingBuffersQueued(0)
    ,mNumCapturegBuffersQueued(0)
    ,mSessionId(0)
    ,mCameraId(cameraId)
{
    LOG1("@%s", __FUNCTION__);

    memset(&mPostviewBuffers, 0, sizeof(mPostviewBuffers));
    memset(&mSnapshotBuffers, 0, sizeof(mSnapshotBuffers));

    mConfig.fps = 30;
    mConfig.num_snapshot = 1;
    mConfig.zoom = 0;

    int ret = openDevice();
    if (ret < 0) {
        LOGE("Failed to open device!");
        return;
    }

    ret = detectDeviceResolutions();
    if (ret) {
        LOGE("Failed to detect camera resolution! Use default settings");
        if (mCameraSensor[cameraId]->info.facing == CAMERA_FACING_FRONT) {
            mConfig.snapshot.maxWidth  = MAX_FRONT_CAMERA_SNAPSHOT_WIDTH;
            mConfig.snapshot.maxHeight = MAX_FRONT_CAMERA_SNAPSHOT_HEIGHT;
        } else {
            mConfig.snapshot.maxWidth  = MAX_BACK_CAMERA_SNAPSHOT_WIDTH;
            mConfig.snapshot.maxHeight = MAX_BACK_CAMERA_SNAPSHOT_HEIGHT;
        }
    } else {
        LOG1("Max-resolution detected: %dx%d",
                mConfig.snapshot.maxWidth,
                mConfig.snapshot.maxHeight);
    }

    if (mCameraSensor[cameraId]->info.facing == CAMERA_FACING_FRONT) {
        mConfig.preview.maxWidth   = MAX_FRONT_CAMERA_PREVIEW_WIDTH;
        mConfig.preview.maxHeight  = MAX_FRONT_CAMERA_PREVIEW_HEIGHT;
        mConfig.recording.maxWidth = MAX_FRONT_CAMERA_VIDEO_WIDTH;
        mConfig.recording.maxHeight = MAX_FRONT_CAMERA_VIDEO_HEIGHT;
    } else {
        mConfig.preview.maxWidth   = MAX_BACK_CAMERA_PREVIEW_WIDTH;
        mConfig.preview.maxHeight  = MAX_BACK_CAMERA_PREVIEW_HEIGHT;
        mConfig.recording.maxWidth = MAX_BACK_CAMERA_VIDEO_WIDTH;
        mConfig.recording.maxHeight = MAX_BACK_CAMERA_VIDEO_HEIGHT;
    }

    // Initialize the frame sizes (TODO: decide on appropriate sizes)
    setPreviewFrameFormat(RESOLUTION_VGA_WIDTH, RESOLUTION_VGA_HEIGHT, V4L2_PIX_FMT_YUYV);
    setPostviewFrameFormat(RESOLUTION_VGA_WIDTH, RESOLUTION_VGA_HEIGHT, V4L2_PIX_FMT_YUYV);
    setSnapshotFrameFormat(RESOLUTION_VGA_WIDTH, RESOLUTION_VGA_HEIGHT, V4L2_PIX_FMT_YUYV);
    setVideoFrameFormat(RESOLUTION_VGA_WIDTH, RESOLUTION_VGA_HEIGHT, V4L2_PIX_FMT_YUYV);

    closeDevice();
}

CameraDriver::~CameraDriver()
{
    LOG1("@%s", __FUNCTION__);
    /*
     * The destructor is called when the hw_module close mehod is called. The close method is called
     * in general by the camera client when it's done with the camera device, but it is also called by
     * System Server when the camera application crashes. System Server calls close in order to release
     * the camera hardware module. So, if we are not in MODE_NONE, it means that we are in the middle of
     * somthing when the close function was called. So it's our duty to stop first, then close the
     * camera device.
     */
    if (mMode != MODE_NONE) {
        stop();
    }
}

void CameraDriver::getDefaultParameters(CameraParameters *params)
{
    LOG2("@%s", __FUNCTION__);
    if (!params) {
        LOGE("params is null!");
        return;
    }

    /**
     * PREVIEW
     */
    params->setPreviewSize(mConfig.preview.width, mConfig.preview.height);
    params->setPreviewFrameRate(30);

    params->set(CameraParameters::KEY_SUPPORTED_PREVIEW_SIZES, "640x480"); // TODO: consider which sizes to support

    params->set(CameraParameters::KEY_SUPPORTED_PREVIEW_FRAME_RATES,"30"); // TODO: consider which FPS to support
    params->set(CameraParameters::KEY_PREVIEW_FPS_RANGE,"30000,30000");
    params->set(CameraParameters::KEY_SUPPORTED_PREVIEW_FPS_RANGE,"(30000,30000)");

    /**
     * RECORDING
     */
    params->setVideoSize(0, 0);
    params->set(CameraParameters::KEY_PREFERRED_PREVIEW_SIZE_FOR_VIDEO, "640x480");
    params->set(CameraParameters::KEY_SUPPORTED_VIDEO_SIZES, ""); // empty string indicates we only support a single stream
    params->set(CameraParameters::KEY_VIDEO_FRAME_FORMAT,
                CameraParameters::PIXEL_FORMAT_YUV420SP);

    params->set(CameraParameters::KEY_VIDEO_SNAPSHOT_SUPPORTED, CameraParameters::FALSE);

    /**
     * SNAPSHOT
     */
    params->set(CameraParameters::KEY_SUPPORTED_PICTURE_SIZES, "640x480"); // TODO: consider which sizes to support
    params->setPictureSize(mConfig.snapshot.width, mConfig.snapshot.height);
    params->set(CameraParameters::KEY_SUPPORTED_JPEG_THUMBNAIL_SIZES,"0x0"); // 0x0 indicates "not supported"

    /**
     * ZOOM
     */
    params->set(CameraParameters::KEY_ZOOM, 0);
    params->set(CameraParameters::KEY_ZOOM_SUPPORTED, CameraParameters::TRUE);
    getZoomRatios(MODE_PREVIEW, params);

    /**
     * FLASH
     */
    params->set(CameraParameters::KEY_FLASH_MODE, CameraParameters::FLASH_MODE_OFF);
    params->set(CameraParameters::KEY_SUPPORTED_FLASH_MODES, CameraParameters::FLASH_MODE_OFF);

    /**
     * FOCUS
     */
    params->set(CameraParameters::KEY_FOCUS_MODE, CameraParameters::FOCUS_MODE_FIXED);
    params->set(CameraParameters::KEY_SUPPORTED_FOCUS_MODES, CameraParameters::FOCUS_MODE_FIXED);

    /**
     * FOCAL LENGTH
     */
    // TODO: find out actual focal length
    // TODO: also find out how to get sensor width and height which will likely be used with focal length
    float focalLength = 0.0; // focalLength unit is mm
    params->setFloat(CameraParameters::KEY_FOCAL_LENGTH, focalLength);

    /**
     * FOCUS DISTANCES
     */
    getFocusDistances(params);

    /**
     * MISCELLANEOUS
     */
    params->set(CameraParameters::KEY_VERTICAL_VIEW_ANGLE,"0.0");
    params->set(CameraParameters::KEY_HORIZONTAL_VIEW_ANGLE,"0.0");

    /**
     * EXPOSURE
     */
    params->set(CameraParameters::KEY_EXPOSURE_COMPENSATION,0);
    params->set(CameraParameters::KEY_MAX_EXPOSURE_COMPENSATION,0);
    params->set(CameraParameters::KEY_MIN_EXPOSURE_COMPENSATION,0);
    params->set(CameraParameters::KEY_EXPOSURE_COMPENSATION_STEP,0);

    // effect modes
    params->set(CameraParameters::KEY_EFFECT, CameraParameters::EFFECT_NONE);
    params->set(CameraParameters::KEY_SUPPORTED_EFFECTS, CameraParameters::EFFECT_NONE);

    // white-balance mode
    params->set(CameraParameters::KEY_WHITE_BALANCE, CameraParameters::WHITE_BALANCE_AUTO);
    params->set(CameraParameters::KEY_SUPPORTED_WHITE_BALANCE, CameraParameters::WHITE_BALANCE_AUTO);

    // scene mode
    params->set(CameraParameters::KEY_SCENE_MODE, CameraParameters::SCENE_MODE_AUTO);
    params->set(CameraParameters::KEY_SUPPORTED_SCENE_MODES, CameraParameters::SCENE_MODE_AUTO);

    // 3a lock: auto-exposure lock
    params->set(CameraParameters::KEY_AUTO_EXPOSURE_LOCK, "");
    params->set(CameraParameters::KEY_AUTO_EXPOSURE_LOCK_SUPPORTED, CameraParameters::FALSE);

    // 3a lock: auto-whitebalance lock
    params->set(CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK, "");
    params->set(CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK_SUPPORTED, CameraParameters::FALSE);

    // multipoint focus
    params->set(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS, 0);

    // set empty area
    params->set(CameraParameters::KEY_FOCUS_AREAS, "");

    // metering areas
    params->set(CameraParameters::KEY_MAX_NUM_METERING_AREAS, 0);
}

status_t CameraDriver::start(Mode mode)
{
    LOG1("@%s", __FUNCTION__);
    LOG1("mode = %d", mode);
    status_t status = NO_ERROR;

    switch (mode) {
    case MODE_PREVIEW:
        status = startPreview();
        break;

    case MODE_VIDEO:
        status = startRecording();
        break;

    case MODE_CAPTURE:
        status = startCapture();
        break;

    default:
        break;
    };

    if (status == NO_ERROR) {
        mMode = mode;
        mSessionId++;
    }

    return status;
}

status_t CameraDriver::stop()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;

    switch (mMode) {
    case MODE_PREVIEW:
        status = stopPreview();
        break;

    case MODE_VIDEO:
        status = stopRecording();
        break;

    case MODE_CAPTURE:
        status = stopCapture();
        break;

    default:
        break;
    };

    if (status == NO_ERROR)
        mMode = MODE_NONE;

    return status;
}

status_t CameraDriver::startPreview()
{
    LOG1("@%s", __FUNCTION__);
    int ret = 0;
    status_t status = NO_ERROR;

    ret = openDevice();
    if (ret < 0) {
        LOGE("Open preview device failed!");
        status = UNKNOWN_ERROR;
        return status;
    }

    if ((status = allocatePreviewBuffers()) != NO_ERROR)
        goto exitClose;

    ret = configureDevice(
            MODE_PREVIEW,
            mConfig.preview.padding,
            mConfig.preview.height,
            mConfig.preview.format,
            false);
    if (ret < 0) {
        LOGE("Configure preview device failed!");
        status = UNKNOWN_ERROR;
        goto exitFreeClose;
    }

    // need to resend the current zoom value
    set_zoom(mCameraSensor[mCameraId]->fd, mConfig.zoom);

    ret = startDevice(mNumBuffers);
    if (ret < 0) {
        LOGE("Start preview device failed!");
        status = UNKNOWN_ERROR;
        goto exitFreeClose;
    }

    mNumPreviewBuffersQueued = mNumBuffers;

    return status;

exitFreeClose:
    freePreviewBuffers();
exitClose:
    closeDevice();
    return status;
}

status_t CameraDriver::stopPreview()
{
    LOG1("@%s", __FUNCTION__);

    freePreviewBuffers();
    stopDevice();
    closeDevice();

    return NO_ERROR;
}

status_t CameraDriver::startRecording() {
    LOG1("@%s", __FUNCTION__);
    int ret = 0;
    status_t status = NO_ERROR;

    ret = openDevice();
    if (ret < 0) {
        LOGE("Open recording device failed!");
        status = UNKNOWN_ERROR;
        return status;
    }

    if ((status = allocateRecordingBuffers()) != NO_ERROR)
        goto exitClose;

    ret = configureDevice(
            MODE_VIDEO,
            mConfig.recording.padding,
            mConfig.recording.height,
            mConfig.recording.format,
            false);
    if (ret < 0) {
        LOGE("Configure recording device failed!");
        status = UNKNOWN_ERROR;
        goto exitFreeClose;
    }

    ret = startDevice(mNumBuffers);
    if (ret < 0) {
        LOGE("Start recording device failed");
        status = UNKNOWN_ERROR;
        goto exitFreeClose;
    }

    mNumPreviewBuffersQueued = mNumBuffers;
    mNumRecordingBuffersQueued = mNumBuffers;

    return status;

exitFreeClose:
    freeRecordingBuffers();
exitClose:
    closeDevice();
    return status;
}

status_t CameraDriver::stopRecording()
{
    LOG1("@%s", __FUNCTION__);

    freeRecordingBuffers();
    stopDevice();
    closeDevice();

    return NO_ERROR;
}

status_t CameraDriver::startCapture()
{
    LOG1("@%s", __FUNCTION__);
    int ret;
    status_t status = NO_ERROR;

    ret = openDevice();
    if (ret < 0) {
        LOGE("Open capture device failed!");
        status = UNKNOWN_ERROR;
        return status;
    }

    if ((status = allocateSnapshotBuffers()) != NO_ERROR)
        goto exitClose;

    ret = configureDevice(
            MODE_CAPTURE,
            mConfig.snapshot.width,
            mConfig.snapshot.height,
            mConfig.snapshot.format,
            false);
    if (ret < 0) {
        LOGE("configure capture device failed!");
        status = UNKNOWN_ERROR;
        goto exitFreeClose;
    }

    // need to resend the current zoom value
    set_zoom(mCameraSensor[mCameraId]->fd, mConfig.zoom);

    ret = startDevice(mConfig.num_snapshot);
    if (ret < 0) {
        LOGE("start capture device failed!");
        status = UNKNOWN_ERROR;
        goto exitFreeClose;
    }

    mNumCapturegBuffersQueued = mConfig.num_snapshot;

    return status;

exitFreeClose:
    freeSnapshotBuffers();
exitClose:
    closeDevice();
    return status;
}

status_t CameraDriver::stopCapture()
{
    LOG1("@%s", __FUNCTION__);

    freeSnapshotBuffers();
    stopDevice();
    closeDevice();

    return NO_ERROR;
}

int CameraDriver::configureDevice(Mode deviceMode, int w, int h, int format, bool raw)
{
    LOG1("@%s", __FUNCTION__);
    int ret = 0;
    LOG1("width:%d, height:%d, deviceMode:%d format:%d raw:%d",
            w, h, deviceMode, format, raw);

    if ((w <= 0) || (h <= 0)) {
        LOGE("Wrong Width %d or Height %d", w, h);
        return -1;
    }

    int fd = mCameraSensor[mCameraId]->fd;

    //Switch the Mode before set the format. This is a driver requirement
    ret = set_capture_mode(deviceMode);
    if (ret < 0)
        return ret;

    //Set the format
    ret = v4l2_capture_s_format(fd, w, h, format, raw);
    if (ret < 0)
        return ret;

    mBufferPool[mCameraId].width = w;
    mBufferPool[mCameraId].height = h;
    mBufferPool[mCameraId].format = format;

    ret = v4l2_capture_g_framerate(fd, &mConfig.fps, w, h, format);
    if (ret < 0) {
        /*Error handler: if driver does not support FPS achieving,
          just give the default value.*/
        mConfig.fps = DEFAULT_SENSOR_FPS;
        ret = 0;
    }

    //We need apply all the parameter settings when do the camera reset
    return ret;
}

int CameraDriver::startDevice(int buffer_count)
{
    LOG1("@%s", __FUNCTION__);

    int i, ret;
    int fd = mCameraSensor[mCameraId]->fd;
    LOG1(" startDevice fd = %d", fd);

    //parameter intialized before the streamon
    //request, query and mmap the buffer and save to the pool
    ret = createBufferPool(buffer_count);
    if (ret < 0)
        return ret;

    //Qbuf
    ret = activateBufferPool();
    if (ret < 0)
        goto activate_error;

    //stream on
    ret = v4l2_capture_streamon(fd);
    if (ret < 0)
       goto streamon_failed;

    //we are started now
    return 0;

streamon_failed:
activate_error:
    destroyBufferPool();
    return ret;
}

int CameraDriver::activateBufferPool()
{
    LOG1("@%s", __FUNCTION__);

    int fd = mCameraSensor[mCameraId]->fd;
    int ret;
    struct DriverBufferPool *pool = &mBufferPool[mCameraId];

    for (int i = 0; i < pool->active_buffers; i++) {
        ret = v4l2_capture_qbuf(fd, i, &pool->bufs[i]);
        if (ret < 0)
            return ret;
    }
    return 0;
}

int CameraDriver::createBufferPool(int buffer_count)
{
    LOG1("@%s", __FUNCTION__);
    int i, ret;

    int fd = mCameraSensor[mCameraId]->fd;
    struct DriverBufferPool *pool = &mBufferPool[mCameraId];
    int num_buffers = v4l2_capture_request_buffers(buffer_count);
    LOG1("num_buffers = %d", num_buffers);

    if (num_buffers <= 0)
        return -1;

    pool->active_buffers = num_buffers;

    for (i = 0; i < num_buffers; i++) {
        pool->bufs[i].width = pool->width;
        pool->bufs[i].height = pool->height;
        pool->bufs[i].format = pool->format;
        ret = v4l2_capture_new_buffer(i, &pool->bufs[i]);
        if (ret < 0)
            goto error;
    }
    return 0;

error:
    for (int j = 0; j < i; j++)
        v4l2_capture_free_buffer(&pool->bufs[j]);
    pool->active_buffers = 0;
    return ret;
}

void CameraDriver::stopDevice()
{
    LOG1("@%s", __FUNCTION__);

    int fd = mCameraSensor[mCameraId]->fd;

    //stream off
    v4l2_capture_streamoff(fd);
    destroyBufferPool();
}

void CameraDriver::destroyBufferPool()
{
    LOG1("@%s", __FUNCTION__);

    int fd = mCameraSensor[mCameraId]->fd;
    struct DriverBufferPool *pool = &mBufferPool[mCameraId];

    for (int i = 0; i < pool->active_buffers; i++)
        v4l2_capture_free_buffer(&pool->bufs[i]);
    pool->active_buffers = 0;
    v4l2_capture_release_buffers();
}

int CameraDriver::openDevice()
{
    int fd;
    LOG1("@%s", __FUNCTION__);
    if (mCameraSensor[mCameraId] == 0) {
        LOGE("%s: Try to open non-existent camera", __FUNCTION__);
        return -ENODEV;
    }

    LOG1("@%s", __FUNCTION__);
    if (mCameraSensor[mCameraId]->fd >= 0) {
        LOGE("%s: camera is already opened", __FUNCTION__);
        return mCameraSensor[mCameraId]->fd;
    }
    const char *dev_name = mCameraSensor[mCameraId]->devName;

    fd = v4l2_capture_open(dev_name);

    if (fd < 0) {
        LOGE("V4L2: capture_open failed: %s", strerror(errno));
        return -EFAULT;
    }

    // Query and check the capabilities
    struct v4l2_capability cap;
    if (v4l2_capture_querycap(fd, &cap) < 0) {
        LOGE("V4L2: capture_querycap failed: %s", strerror(errno));
        v4l2_capture_close(fd);
        return -EFAULT;
    }

    mCameraSensor[mCameraId]->fd = fd;

    return mCameraSensor[mCameraId]->fd;
}

void CameraDriver::closeDevice()
{
    LOG1("@%s", __FUNCTION__);

    if (mCameraSensor[mCameraId] == 0) {
        LOGE("%s: Try to open non-existent camera", __FUNCTION__);
        return;
    }

    if (mCameraSensor[mCameraId]->fd < 0)
        return;

    v4l2_capture_close(mCameraSensor[mCameraId]->fd);

    mCameraSensor[mCameraId]->fd = -1;
}


int CameraDriver::detectDeviceResolutions()
{
    LOG1("@%s", __FUNCTION__);
    int ret = 0;
    struct v4l2_frmsizeenum frame_size;

    //Switch the Mode before try the format.
    ret = set_capture_mode(MODE_CAPTURE);
    if (ret < 0)
        return ret;

    int i = 0;
    while (true) {
        memset(&frame_size, 0, sizeof(frame_size));
        frame_size.index = i++;
        frame_size.pixel_format = mConfig.snapshot.format;
        /* TODO: Currently VIDIOC_ENUM_FRAMESIZES is returning with Invalid argument
         * Need to know why the driver is not supporting this V4L2 API call
         */
        if (ioctl(mCameraSensor[mCameraId]->fd, VIDIOC_ENUM_FRAMESIZES, &frame_size) < 0) {
            break;
        }
        ret++;
        float fps = 0;
        v4l2_capture_g_framerate(
                mCameraSensor[mCameraId]->fd,
                &fps,
                frame_size.discrete.width,
                frame_size.discrete.height,
                frame_size.pixel_format);
        LOG1("Supported frame size: %ux%u@%dfps",
                frame_size.discrete.width,
                frame_size.discrete.height,
                static_cast<int>(fps));
    }

    // Get the maximum format supported
    mConfig.snapshot.maxWidth = 0xffff;
    mConfig.snapshot.maxHeight = 0xffff;
    ret = v4l2_capture_try_format(mCameraSensor[mCameraId]->fd, &mConfig.snapshot.maxWidth, &mConfig.snapshot.maxHeight, &mConfig.snapshot.format);
    if (ret < 0)
        return ret;
    return 0;
}

status_t CameraDriver::setPreviewFrameFormat(int width, int height, int format)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;

    if(format == 0)
         format = mConfig.preview.format;
    if (width > mConfig.preview.maxWidth || width <= 0)
        width = mConfig.preview.maxWidth;
    if (height > mConfig.preview.maxHeight || height <= 0)
        height = mConfig.preview.maxHeight;
    mConfig.preview.width = width;
    mConfig.preview.height = height;
    mConfig.preview.format = format;
    mConfig.preview.padding = paddingWidth(format, width, height);
    mConfig.preview.size = frameSize(format, mConfig.preview.padding, height);
    LOG1("width(%d), height(%d), pad_width(%d), size(%d), format(%x)",
        width, height, mConfig.preview.padding, mConfig.preview.size, format);
    return status;
}

status_t CameraDriver::setPostviewFrameFormat(int width, int height, int format)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;

    LOG1("width(%d), height(%d), format(%x)",
         width, height, format);
    mConfig.postview.width = width;
    mConfig.postview.height = height;
    mConfig.postview.format = format;
    mConfig.postview.padding = paddingWidth(format, width, height);
    mConfig.postview.size = frameSize(format, width, height);
    if (mConfig.postview.size == 0)
        mConfig.postview.size = mConfig.postview.width * mConfig.postview.height * BPP;
    LOG1("width(%d), height(%d), pad_width(%d), size(%d), format(%x)",
            width, height, mConfig.postview.padding, mConfig.postview.size, format);
    return status;
}

status_t CameraDriver::setSnapshotFrameFormat(int width, int height, int format)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;

    if (width > mConfig.snapshot.maxWidth || width <= 0)
        width = mConfig.snapshot.maxWidth;
    if (height > mConfig.snapshot.maxHeight || height <= 0)
        height = mConfig.snapshot.maxHeight;
    mConfig.snapshot.width  = width;
    mConfig.snapshot.height = height;
    mConfig.snapshot.format = format;
    mConfig.snapshot.padding = paddingWidth(format, width, height);
    mConfig.snapshot.size = frameSize(format, width, height);;
    if (mConfig.snapshot.size == 0)
        mConfig.snapshot.size = mConfig.snapshot.width * mConfig.snapshot.height * BPP;
    LOG1("width(%d), height(%d), pad_width(%d), size(%d), format(%x)",
        width, height, mConfig.snapshot.padding, mConfig.snapshot.size, format);
    return status;
}

void CameraDriver::getVideoSize(int *width, int *height)
{
    if (width && height) {
        *width = mConfig.recording.width;
        *height = mConfig.recording.height;
    }
}

status_t CameraDriver::setVideoFrameFormat(int width, int height, int format)
{
    LOG1("@%s", __FUNCTION__);
    int ret = 0;
    status_t status = NO_ERROR;

    if(format == 0)
         format = mConfig.recording.format;
    if (mConfig.recording.width == width &&
        mConfig.recording.height == height &&
        mConfig.recording.format == format) {
        // Do nothing
        return status;
    }

    if (mMode == MODE_VIDEO) {
        LOGE("Reconfiguration in video mode unsupported. Stop the driver first");
        return INVALID_OPERATION;
    }

    if (width > mConfig.recording.maxWidth || width <= 0) {
        LOGE("invalid recording width %d. override to %d", width, mConfig.recording.maxWidth);
        width = mConfig.recording.maxWidth;
    }
    if (height > mConfig.recording.maxHeight || height <= 0) {
        LOGE("invalid recording height %d. override to %d", height, mConfig.recording.maxHeight);
        height = mConfig.recording.maxHeight;
    }
    mConfig.recording.width = width;
    mConfig.recording.height = height;
    mConfig.recording.format = format;
    mConfig.recording.padding = paddingWidth(format, width, height);
    mConfig.recording.size = frameSize(format, width, height);
    if (mConfig.recording.size == 0)
        mConfig.recording.size = mConfig.recording.width * mConfig.recording.height * BPP;
    LOG1("width(%d), height(%d), pad_width(%d), format(%x)",
            width, height, mConfig.recording.padding, format);

    return status;
}

void CameraDriver::getZoomRatios(Mode mode, CameraParameters *params)
{
    LOG1("@%s", __FUNCTION__);

    // TODO: decide if we can support zoom

    // zoom is not supported. this is indicated by placing a single zoom ratio in params
    params->set(CameraParameters::KEY_MAX_ZOOM, "0"); // zoom index 0 indicates first (and only) zoom ratio
    params->set(CameraParameters::KEY_ZOOM_RATIOS, "100");
}

void CameraDriver::getFocusDistances(CameraParameters *params)
{
    LOG1("@%s", __FUNCTION__);
    // TODO: set focus distances (CameraParameters::KEY_FOCUS_DISTANCES,)
}

status_t CameraDriver::setZoom(int zoom)
{
    LOG1("@%s: zoom = %d", __FUNCTION__, zoom);
    if (zoom == mConfig.zoom)
        return NO_ERROR;
    if (mMode == MODE_CAPTURE)
        return NO_ERROR;

    int ret = set_zoom(mCameraSensor[mCameraId]->fd, zoom);
    if (ret < 0) {
        LOGE("Error setting zoom to %d", zoom);
        return UNKNOWN_ERROR;
    }
    mConfig.zoom = zoom;
    return NO_ERROR;
}

status_t CameraDriver::getFNumber(unsigned int *fNumber)
{
    LOG1("@%s", __FUNCTION__);

    // TODO: implement

    return NO_ERROR;
}

status_t CameraDriver::getExposureInfo(CamExifExposureProgramType *exposureProgram,
                                       CamExifExposureModeType *exposureMode,
                                       int *exposureTime,
                                       float *exposureBias,
                                       int *aperture)
{
    // TODO: implement
    return INVALID_OPERATION;
}

status_t CameraDriver::getBrightness(float *brightness)
{
    // TODO: implement
    return INVALID_OPERATION;
}

status_t CameraDriver::getIsoSpeed(int *isoSpeed)
{
    // TODO: implement
    return INVALID_OPERATION;
}

status_t CameraDriver::getMeteringMode(CamExifMeteringModeType *meteringMode)
{
    // TODO: implement
    return INVALID_OPERATION;
}

status_t CameraDriver::getAWBMode(CamExifWhiteBalanceType *wbMode)
{
    // TODO: implement
    return INVALID_OPERATION;
}

status_t CameraDriver::getSceneMode(CamExifSceneCaptureType *sceneMode)
{
    // TODO: implement
    return INVALID_OPERATION;
}

int CameraDriver::set_zoom(int fd, int zoom)
{
    LOG1("@%s", __FUNCTION__);

    // TODO: set zoom

    return NO_ERROR;
}

int CameraDriver::set_attribute (int fd, int attribute_num,
                                             const int value, const char *name)
{
    LOG1("@%s", __FUNCTION__);
    struct v4l2_control control;
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control ext_control;

    LOG1("setting attribute [%s] to %d", name, value);

    if (fd < 0)
        return -1;

    control.id = attribute_num;
    control.value = value;
    controls.ctrl_class = V4L2_CTRL_CLASS_CAMERA;
    controls.count = 1;
    controls.controls = &ext_control;
    ext_control.id = attribute_num;
    ext_control.value = value;

    if (ioctl(fd, VIDIOC_S_CTRL, &control) == 0)
        return 0;

    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) == 0)
        return 0;

    controls.ctrl_class = V4L2_CTRL_CLASS_USER;
    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) == 0)
        return 0;

    LOGE("Failed to set value %d for control %s (%d) on fd '%d', %s",
        value, name, attribute_num, fd, strerror(errno));
    return -1;
}

int CameraDriver::xioctl(int fd, int request, void *arg)
{
    int ret;

    do {
        ret = ioctl (fd, request, arg);
    } while (-1 == ret && EINTR == errno);

    if (ret < 0)
        LOGW ("Request %d failed: %s", request, strerror(errno));

    return ret;
}

int CameraDriver::v4l2_capture_streamon(int fd)
{
    LOG1("@%s", __FUNCTION__);
    int ret;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    ret = ioctl(fd, VIDIOC_STREAMON, &type);
    if (ret < 0) {
        LOGE("VIDIOC_STREAMON returned: %d (%s)", ret, strerror(errno));
        return ret;
    }
    return ret;
}

int CameraDriver::v4l2_capture_streamoff(int fd)
{
    LOG1("@%s", __FUNCTION__);
    int ret;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (fd < 0){ //Device is closed
        LOGE("Device is closed!");
        return 0;
    }
    ret = ioctl(fd, VIDIOC_STREAMOFF, &type);
    if (ret < 0) {
        LOGE("VIDIOC_STREAMOFF returned: %d (%s)", ret, strerror(errno));
        return ret;
    }

    return ret;
}

/* Unmap the buffer or free the userptr */
int CameraDriver::v4l2_capture_free_buffer(struct DriverBuffer *buf_info)
{
    LOG1("@%s", __FUNCTION__);
    int ret = 0;
    void *addr = buf_info->data;
    size_t length = buf_info->length;

    // FIXME: Temporary solution for mmap streams. We should support user ptr and read/write IO
    if ((ret = munmap(addr, length)) < 0) {
            LOGE("munmap returned: %d (%s)", ret, strerror(errno));
            return ret;
    }

    return ret;
}

int CameraDriver::v4l2_capture_release_buffers()
{
    LOG1("@%s", __FUNCTION__);
    return v4l2_capture_request_buffers(0);
}

int CameraDriver::v4l2_capture_request_buffers(uint num_buffers)
{
    LOG1("@%s", __FUNCTION__);
    struct v4l2_requestbuffers req_buf;
    int ret;
    CLEAR(req_buf);

    int fd = mCameraSensor[mCameraId]->fd;

    if (fd < 0)
        return 0;

    req_buf.memory = V4L2_MEMORY_USERPTR;
    req_buf.count = num_buffers;
    req_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // TODO: Temporary solution for 1st camera and mmap streams.
    if (mCameraId == 0) {
        req_buf.memory = V4L2_MEMORY_MMAP;
        req_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    }

    LOG1("VIDIOC_REQBUFS, count=%d", req_buf.count);
    ret = ioctl(fd, VIDIOC_REQBUFS, &req_buf);

    if (ret < 0) {
        LOGE("VIDIOC_REQBUFS(%d) returned: %d (%s)",
            num_buffers, ret, strerror(errno));
        return ret;
    }

    if (req_buf.count < num_buffers)
        LOGW("Got less buffers than requested!");

    return req_buf.count;
}

int CameraDriver::v4l2_capture_new_buffer(int index, struct DriverBuffer *buf)
{
    LOG1("@%s", __FUNCTION__);
    void *data;
    int ret;
    int fd = mCameraSensor[mCameraId]->fd;
    struct v4l2_buffer *vbuf = &buf->vbuffer;
    vbuf->flags = 0x0;

    // TODO: Temporary solution for 1st camera and mmap streams.
    if (mCameraId == 0) {
        vbuf->index = index;
        vbuf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vbuf->memory = V4L2_MEMORY_MMAP;
        ret = ioctl(fd, VIDIOC_QUERYBUF, vbuf);
        if (ret < 0) {
            LOGE("VIDIOC_QUERYBUF failed: %s", strerror(errno));
            return -1;
        }
        data = mmap(NULL, vbuf->length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                vbuf->m.offset);
        if (MAP_FAILED == data) {
            LOGE("mmap failed: %s", strerror(errno));
            return -1;
        }
        buf->data = data;
        buf->length = vbuf->length;

        //FIXME: we change to mmap buffer that will be provided by kernel, instead
        //the old design  to allocate a user space buffer for kernel. So we setup
        //pointers in a reversed way...

        // FIXME: WE assume num of preview buffers are same as pool buffers...
        mPreviewBuffers[index].buff->data = data;
        mPreviewBuffers[index].buff->size = vbuf->length;

        LOG1("!! Setup mmap for 1st dev at %p with len %d and mPreviewBuffer[%d]", buf->data, buf->length, index);
        return 0;
    }

    vbuf->memory = V4L2_MEMORY_USERPTR;

    vbuf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    vbuf->index = index;
    ret = ioctl(fd , VIDIOC_QUERYBUF, vbuf);

    if (ret < 0) {
        LOGE("VIDIOC_QUERYBUF failed: %s", strerror(errno));
        return ret;
    }

    vbuf->m.userptr = (unsigned int)(buf->data);

    buf->length = vbuf->length;
    LOG1("index %u", vbuf->index);
    LOG1("type %d", vbuf->type);
    LOG1("bytesused %u", vbuf->bytesused);
    LOG1("flags %08x", vbuf->flags);
    LOG1("memory %u", vbuf->memory);
    LOG1("userptr:  %lu", vbuf->m.userptr);
    LOG1("length %u", vbuf->length);
    LOG1("input %u", vbuf->input);
    return ret;
}

int CameraDriver::v4l2_capture_g_framerate(int fd, float *framerate, int width,
                                         int height, int pix_fmt)
{
    LOG1("@%s", __FUNCTION__);
    int ret;
    struct v4l2_frmivalenum frm_interval;

    if (NULL == framerate)
        return -EINVAL;

    assert(fd > 0);
    CLEAR(frm_interval);
    frm_interval.pixel_format = pix_fmt;
    frm_interval.width = width;
    frm_interval.height = height;
    *framerate = -1.0;

    ret = ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frm_interval);
    if (ret < 0) {
        LOGW("ioctl failed: %s", strerror(errno));
        return ret;
    }

    assert(0 != frm_interval.discrete.denominator);

    *framerate = 1.0 / (1.0 * frm_interval.discrete.numerator / frm_interval.discrete.denominator);

    return 0;
}

int CameraDriver::v4l2_capture_s_format(int fd, int w, int h, int fourcc, bool raw)
{
    LOG1("@%s", __FUNCTION__);
    int ret;
    struct v4l2_format v4l2_fmt;
    CLEAR(v4l2_fmt);

    v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    LOG1("VIDIOC_G_FMT");
    ret = ioctl (fd,  VIDIOC_G_FMT, &v4l2_fmt);
    if (ret < 0) {
        LOGE("VIDIOC_G_FMT failed: %s", strerror(errno));
        return -1;
    }
    if (raw) {
        LOG1("Choose raw dump path");
        v4l2_fmt.type = V4L2_BUF_TYPE_PRIVATE;
    } else {
        v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    }

    v4l2_fmt.fmt.pix.width = w;
    v4l2_fmt.fmt.pix.height = h;
    v4l2_fmt.fmt.pix.pixelformat = fourcc;
    v4l2_fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    LOG1("VIDIOC_S_FMT: width: %d, height: %d, format: %d, field: %d",
                v4l2_fmt.fmt.pix.width,
                v4l2_fmt.fmt.pix.height,
                v4l2_fmt.fmt.pix.pixelformat,
                v4l2_fmt.fmt.pix.field);
    ret = ioctl(fd, VIDIOC_S_FMT, &v4l2_fmt);
    if (ret < 0) {
        LOGE("VIDIOC_S_FMT failed: %s", strerror(errno));
        return -1;
    }
    return 0;

}

int CameraDriver::v4l2_capture_qbuf(int fd, int index, struct DriverBuffer *buf)
{
    LOG2("@%s", __FUNCTION__);
    struct v4l2_buffer *v4l2_buf = &buf->vbuffer;
    int ret;

    if (fd < 0) //Device is closed
        return 0;
    ret = ioctl(fd, VIDIOC_QBUF, v4l2_buf);
    if (ret < 0) {
        LOGE("VIDIOC_QBUF index %d failed: %s",
             index, strerror(errno));
        return ret;
    }
    return ret;
}

status_t CameraDriver::v4l2_capture_open(const char *devName)
{
    LOG1("@%s", __FUNCTION__);
    int fd;
    struct stat st;

    LOG1("---Open video device %s---", devName);

    if (stat (devName, &st) == -1) {
        LOGE("Error stat video device %s: %s",
                devName, strerror(errno));
        return -1;
    }

    if (!S_ISCHR (st.st_mode)) {
        LOGE("%s is not a device", devName);
        return -1;
    }

    fd = open(devName, O_RDWR);

    if (fd <= 0) {
        LOGE("Error opening video device %s: %s",
                devName, strerror(errno));
        return -1;
    }

    return fd;
}

status_t CameraDriver::v4l2_capture_close(int fd)
{
    LOG1("@%s", __FUNCTION__);
    /* close video device */
    LOG1("----close device ---");
    if (fd < 0) {
        LOGW("Device not opened!");
        return INVALID_OPERATION;
    }

    if (close(fd) < 0) {
        LOGE("Close video device failed: %s", strerror(errno));
        return UNKNOWN_ERROR;
    }
    return NO_ERROR;
}

status_t CameraDriver::v4l2_capture_querycap(int fd, struct v4l2_capability *cap)
{
    LOG1("@%s", __FUNCTION__);
    int ret = 0;

    ret = ioctl(fd, VIDIOC_QUERYCAP, cap);

    if (ret < 0) {
        LOGE("VIDIOC_QUERYCAP returned: %d (%s)", ret, strerror(errno));
        return ret;
    }

    if (!(cap->capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        LOGE("No capture devices");
        return -1;
    }

    if (!(cap->capabilities & V4L2_CAP_STREAMING)) {
        LOGE("Is not a video streaming device");
        return -1;
    }

    LOG1( "driver:      '%s'", cap->driver);
    LOG1( "card:        '%s'", cap->card);
    LOG1( "bus_info:      '%s'", cap->bus_info);
    LOG1( "version:      %x", cap->version);
    LOG1( "capabilities:      %x", cap->capabilities);

    return ret;
}

status_t CameraDriver::v4l2_capture_s_input(int fd, int index)
{
    LOG1("@%s", __FUNCTION__);
    struct v4l2_input input;
    int ret;

    LOG1("VIDIOC_S_INPUT");
    input.index = index;

    ret = ioctl(fd, VIDIOC_S_INPUT, &input);

    if (ret < 0) {
        LOGE("VIDIOC_S_INPUT index %d returned: %d (%s)",
            input.index, ret, strerror(errno));
        return ret;
    }
    return ret;
}

int CameraDriver::set_capture_mode(Mode deviceMode)
{
    LOG1("@%s", __FUNCTION__);
    struct v4l2_streamparm parm;

    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.capturemode = deviceMode;
    LOG1("%s !! camID %d fd %d", __FUNCTION__, mCameraId, mCameraSensor[mCameraId]->fd);
    if (ioctl(mCameraSensor[mCameraId]->fd, VIDIOC_S_PARM, &parm) < 0) {
        LOGE("error %s", strerror(errno));
        return -1;
    }

    return 0;
}

int CameraDriver::v4l2_capture_try_format(int fd, int *w, int *h,
                                         int *fourcc)
{
    LOG1("@%s", __FUNCTION__);
    int ret;
    struct v4l2_format v4l2_fmt;
    CLEAR(v4l2_fmt);

    v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    v4l2_fmt.fmt.pix.width = *w;
    v4l2_fmt.fmt.pix.height = *h;
    v4l2_fmt.fmt.pix.pixelformat = *fourcc;
    v4l2_fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

    ret = ioctl(fd, VIDIOC_TRY_FMT, &v4l2_fmt);
    if (ret < 0) {
        LOGE("VIDIOC_TRY_FMT returned: %d (%s)", ret, strerror(errno));
        return -1;
    }

    *w = v4l2_fmt.fmt.pix.width;
    *h = v4l2_fmt.fmt.pix.height;
    *fourcc = v4l2_fmt.fmt.pix.pixelformat;

    return 0;
}

status_t CameraDriver::getPreviewFrame(CameraBuffer *buff)
{
    LOG2("@%s", __FUNCTION__);
    struct v4l2_buffer buf;

    if (mMode == MODE_NONE)
        return INVALID_OPERATION;

    int index = grabFrame(&buf);
    if(index < 0){
        LOGE("Error in grabbing frame!");
        return BAD_INDEX;
    }
    LOG2("Grabbed frame of size: %d", buf.bytesused);
    mPreviewBuffers[index].id = index;
    mPreviewBuffers[index].driverPrivate = mSessionId;
    *buff = mPreviewBuffers[index];

    mNumPreviewBuffersQueued--;

    return NO_ERROR;
}

status_t CameraDriver::putPreviewFrame(CameraBuffer *buff)
{
    LOG2("@%s", __FUNCTION__);
    if (mMode == MODE_NONE)
        return INVALID_OPERATION;

    if (buff->driverPrivate != mSessionId)
        return DEAD_OBJECT;

    if (v4l2_capture_qbuf(mCameraSensor[mCameraId]->fd,
                      buff->id,
                      &mBufferPool[mCameraId].bufs[buff->id]) < 0) {
        return UNKNOWN_ERROR;
    }

    mNumPreviewBuffersQueued++;

    return NO_ERROR;
}

status_t CameraDriver::getRecordingFrame(CameraBuffer *buff, nsecs_t *timestamp)
{
    LOG2("@%s", __FUNCTION__);
    struct v4l2_buffer buf;

    if (mMode != MODE_VIDEO)
        return INVALID_OPERATION;

    int index = grabFrame(&buf);
    LOG2("index = %d", index);
    if(index < 0) {
        LOGE("Error in grabbing frame!");
        return BAD_INDEX;
    }
    LOG2("Grabbed frame of size: %d", buf.bytesused);
    mRecordingBuffers[index].id = index;
    mRecordingBuffers[index].driverPrivate = mSessionId;
    *buff = mRecordingBuffers[index];
    *timestamp = systemTime();

    mNumRecordingBuffersQueued--;

    return NO_ERROR;
}

status_t CameraDriver::putRecordingFrame(CameraBuffer *buff)
{
    LOG2("@%s", __FUNCTION__);
    if (mMode != MODE_VIDEO)
        return INVALID_OPERATION;

    if (buff->driverPrivate != mSessionId)
        return DEAD_OBJECT;

    if (v4l2_capture_qbuf(mCameraSensor[mCameraId]->fd,
            buff->id,
            &mBufferPool[mCameraId].bufs[buff->id]) < 0) {
        return UNKNOWN_ERROR;
    }

    mNumRecordingBuffersQueued++;

    return NO_ERROR;
}

status_t CameraDriver::getSnapshot(CameraBuffer *buff)
{
    LOG1("@%s", __FUNCTION__);
    struct v4l2_buffer buf;
    int index;

    if (mMode != MODE_CAPTURE)
        return INVALID_OPERATION;

    index = grabFrame(&buf);
    if (index < 0) {
        LOGE("Error in grabbing frame from 1'st device!");
        return BAD_INDEX;
    }
    LOG1("Grabbed frame of size: %d", buf.bytesused);

    mSnapshotBuffers[index].id = index;
    mSnapshotBuffers[index].driverPrivate = mSessionId;
    *buff = mSnapshotBuffers[index];

    mNumCapturegBuffersQueued--;

    return NO_ERROR;
}

status_t CameraDriver::putSnapshot(CameraBuffer *buff)
{
    LOG1("@%s", __FUNCTION__);
    int ret;

    if (mMode != MODE_CAPTURE)
        return INVALID_OPERATION;

    if (buff->driverPrivate != mSessionId)
        return DEAD_OBJECT;

    ret = v4l2_capture_qbuf(mCameraSensor[mCameraId]->fd, buff->id,
                      &mBufferPool[mCameraId].bufs[buff->id]);

    if (ret < 0)
        return UNKNOWN_ERROR;

    mNumCapturegBuffersQueued++;

    return NO_ERROR;
}

status_t CameraDriver::getThumbnail(CameraBuffer *buff)
{
    LOG1("@%s", __FUNCTION__);
    return INVALID_OPERATION;
}

status_t CameraDriver::putThumbnail(CameraBuffer *buff)
{
    LOG1("@%s", __FUNCTION__);
    return INVALID_OPERATION;
}

bool CameraDriver::dataAvailable()
{
    LOG2("@%s", __FUNCTION__);

    // For video/recording, make sure driver has a preview and a recording buffer
    if (mMode == MODE_VIDEO)
        return mNumRecordingBuffersQueued > 0 && mNumPreviewBuffersQueued > 0;

    // For capture, just make sure driver has a capture buffer
    if (mMode == MODE_CAPTURE)
        return mNumCapturegBuffersQueued > 0;

    // For preview, just make sure driver has a preview buffer
    if (mMode == MODE_PREVIEW)
        return mNumPreviewBuffersQueued > 0;

    LOGE("Query for data in invalid mode");

    return false;
}

bool CameraDriver::isBufferValid(const CameraBuffer* buffer) const
{
    return buffer->driverPrivate == this->mSessionId;
}

int CameraDriver::grabFrame(struct v4l2_buffer *buf)
{
    LOG2("@%s", __FUNCTION__);
    int ret;

    // Temporary solution for mmap streams.
    buf->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf->memory = V4L2_MEMORY_MMAP;

    ret = v4l2_capture_dqbuf(mCameraSensor[mCameraId]->fd, buf);

    if (ret < 0)
        return ret;

    return buf->index;
}

int CameraDriver::v4l2_capture_dqbuf(int fd, struct v4l2_buffer *buf)
{
    LOG2("@%s", __FUNCTION__);
    int ret;

    ret = ioctl(fd, VIDIOC_DQBUF, buf);

    if (ret < 0) {
        LOGE("error dequeuing buffers");
        return ret;
    }

    return buf->index;
}

////////////////////////////////////////////////////////////////////
//                          PRIVATE METHODS
////////////////////////////////////////////////////////////////////

status_t CameraDriver::allocatePreviewBuffers()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    int allocatedBufs = 0;

    int size = frameSize(mConfig.preview.format,
            mConfig.preview.padding,
            mConfig.preview.height);

    mPreviewBuffers = new CameraBuffer[mNumBuffers];
    if (!mPreviewBuffers) {
        LOGE("Not enough mem for preview buffer array");
        status = NO_MEMORY;
        goto errorFree;
    }

    LOG1("Allocating %d buffers of size %d", mNumBuffers, size);
    for (int i = 0; i < mNumBuffers; i++) {
         mPreviewBuffers[i].buff = NULL;
         mCallbacks->allocateMemory(&mPreviewBuffers[i],  mConfig.preview.size);
         if (mPreviewBuffers[i].buff == NULL) {
             LOGE("Error allocation memory for preview buffers!");
             status = NO_MEMORY;
             goto errorFree;
         }
         allocatedBufs++;
         mBufferPool[mCameraId].bufs[i].data = mPreviewBuffers[i].buff->data;
    }
    return status;

errorFree:
    // On error, free the allocated buffers
    for (int i = 0 ; i < allocatedBufs; i++) {
        if (mRecordingBuffers[i].buff != NULL) {
            mRecordingBuffers[i].buff->release(mRecordingBuffers[i].buff);
            mRecordingBuffers[i].buff = NULL;
        }
    }
    if (mPreviewBuffers)
        delete [] mPreviewBuffers;
    return status;
}

status_t CameraDriver::allocateRecordingBuffers()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    int allocatedBufs = 0;
    int size;

    size = mConfig.recording.width * mConfig.recording.height * 3 / 2;

    mRecordingBuffers = new CameraBuffer[mNumBuffers];
    if (!mRecordingBuffers) {
        LOGE("Not enough mem for recording buffer array");
        status = NO_MEMORY;
        goto errorFree;
    }

    for (int i = 0; i < mNumBuffers; i++) {
        mRecordingBuffers[i].buff = NULL;
        mCallbacks->allocateMemory(&mRecordingBuffers[i], size);
        LOG1("allocate recording buffer[%d] buff=%p size=%d",
                i,
                mRecordingBuffers[i].buff->data,
                mRecordingBuffers[i].buff->size);
        if (mRecordingBuffers[i].buff == NULL) {
            LOGE("Error allocation memory for recording buffers!");
            status = NO_MEMORY;
            goto errorFree;
        }
        allocatedBufs++;
        mBufferPool[mCameraId].bufs[i].data = mRecordingBuffers[i].buff->data;
    }
    return status;

errorFree:
    // On error, free the allocated buffers
    for (int i = 0 ; i < allocatedBufs; i++) {
        if (mRecordingBuffers[i].buff != NULL) {
            mRecordingBuffers[i].buff->release(mRecordingBuffers[i].buff);
            mRecordingBuffers[i].buff = NULL;
        }
    }
    if (mRecordingBuffers)
        delete [] mRecordingBuffers;
    return status;
}

status_t CameraDriver::allocateSnapshotBuffers()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    int allocatedSnaphotBufs = 0;
    int allocatedPostviewBufs = 0;
    int snapshotSize = mConfig.snapshot.size;

    LOG1("Allocating %d buffers of size: %d (snapshot), %d (postview)",
            mConfig.num_snapshot,
            snapshotSize,
            mConfig.postview.size);
    for (int i = 0; i < mConfig.num_snapshot; i++) {
        mSnapshotBuffers[i].buff = NULL;
        mCallbacks->allocateMemory(&mSnapshotBuffers[i], snapshotSize);
        if (mSnapshotBuffers[i].buff == NULL) {
            LOGE("Error allocation memory for snapshot buffers!");
            status = NO_MEMORY;
            goto errorFree;
        }
        allocatedSnaphotBufs++;
        mBufferPool[mCameraId].bufs[i].data = mSnapshotBuffers[i].buff->data;

        mPostviewBuffers[i].buff = NULL;
        mCallbacks->allocateMemory(&mPostviewBuffers[i], mConfig.postview.size);
        if (mPostviewBuffers[i].buff == NULL) {
            LOGE("Error allocation memory for postview buffers!");
            status = NO_MEMORY;
            goto errorFree;
        }
        allocatedPostviewBufs++;
        mBufferPool[mCameraId].bufs[i].data = mPostviewBuffers[i].buff->data;
    }
    return status;

errorFree:
    // On error, free the allocated buffers
    for (int i = 0 ; i < allocatedSnaphotBufs; i++) {
        if (mSnapshotBuffers[i].buff != NULL) {
            mSnapshotBuffers[i].buff->release(mSnapshotBuffers[i].buff);
            mSnapshotBuffers[i].buff = NULL;
        }
    }
    for (int i = 0 ; i < allocatedPostviewBufs; i++) {
        if (mPostviewBuffers[i].buff != NULL) {
            mPostviewBuffers[i].buff->release(mPostviewBuffers[i].buff);
            mPostviewBuffers[i].buff = NULL;
        }
    }
    return status;
}

status_t CameraDriver::freePreviewBuffers()
{
    LOG1("@%s", __FUNCTION__);
    for (int i = 0 ; i < mNumBuffers; i++) {
        if (mPreviewBuffers[i].buff != NULL) {
            mPreviewBuffers[i].buff->release(mPreviewBuffers[i].buff);
            mPreviewBuffers[i].buff = NULL;
        }
    }
    delete [] mPreviewBuffers;
    return NO_ERROR;
}

status_t CameraDriver::freeRecordingBuffers()
{
    LOG1("@%s", __FUNCTION__);
    for (int i = 0 ; i < mNumBuffers; i++) {
        if (mRecordingBuffers[i].buff != NULL) {
            mRecordingBuffers[i].buff->release(mRecordingBuffers[i].buff);
            mRecordingBuffers[i].buff = NULL;
        }
    }
    delete [] mRecordingBuffers;
    return NO_ERROR;
}

status_t CameraDriver::freeSnapshotBuffers()
{
    LOG1("@%s", __FUNCTION__);
    for (int i = 0 ; i < mConfig.num_snapshot; i++) {
        if (mSnapshotBuffers[i].buff != NULL) {
            mSnapshotBuffers[i].buff->release(mSnapshotBuffers[i].buff);
            mSnapshotBuffers[i].buff = NULL;
        }
        if (mPostviewBuffers[i].buff != NULL) {
            mPostviewBuffers[i].buff->release(mPostviewBuffers[i].buff);
            mPostviewBuffers[i].buff = NULL;
        }
    }
    return NO_ERROR;
}

int CameraDriver::getNumberOfCameras()
{
    LOG1("@%s", __FUNCTION__);
    return CameraDriver::enumerateCameras();
}

status_t CameraDriver::getCameraInfo(int cameraId, camera_info *cameraInfo)
{
    LOG1("@%s: cameraId = %d", __FUNCTION__, cameraId);
    if (cameraId >= MAX_CAMERAS || cameraId < 0 || mCameraSensor[cameraId] == 0)
        return BAD_VALUE;

    memcpy(cameraInfo, &mCameraSensor[cameraId]->info, sizeof(camera_info));
    LOG1("%s: cameraId = %d, %s, %d", __FUNCTION__, cameraId,
            cameraInfo->facing == CAMERA_FACING_FRONT ? "front" : "back",
            cameraInfo->orientation);
    return NO_ERROR;
}

// Prop definitions.
static const char *PROP_PREFIX = "ro.camera";
static const char *PROP_NUMBER = "number";
static const char *PROP_DEVNAME = "devname";
static const char *PROP_FACING = "facing";
static const char *PROP_ORIENTATION = "orientation";
static const char *PROP_FACING_FRONT = "front";
static const char *PROP_FACING_BACK = "back";

// This function could be called from HAL's get_number_of_cameras() interface
// even before any CameraDriver's instance is created. For ANY errors, we will
// return 0 cameras detected to Android.
int CameraDriver::enumerateCameras(){
    int terminated = 0;
    static struct CameraSensor *newDev;
    int claimed;
    LOG1("@%s", __FUNCTION__);

    Mutex::Autolock _l(mCameraSensorLock);

    // clean up old enumeration.
    cleanupCameras();

    //start a new enumeration for all cameras

    char propKey[PROPERTY_KEY_MAX];
    char propVal[PROPERTY_VALUE_MAX];

    // get total number of cameras
    snprintf(propKey, sizeof(propKey), "%s.%s", PROP_PREFIX, PROP_NUMBER);
    if (0 == property_get(propKey, propVal, 0)) {
        LOGE("%s: Failed to get number of cameras from prop.", __FUNCTION__);
        goto abort;
    }

    claimed = atoi(propVal);

    if (claimed < 0) {
        LOGE("%s: Invalid Claimed (%d) camera(s), abort.", __FUNCTION__, claimed);
        goto abort;
    }

    if (claimed > MAX_CAMERAS) {
        LOGD("%s: Claimed (%d) camera(s), but we only support up to (%d) camera(s)",
                __FUNCTION__, claimed, MAX_CAMERAS);
        claimed = MAX_CAMERAS;
    }

    for (int i = 0; i < claimed; i++) {
        newDev = new CameraSensor;
        if (!newDev) {
            LOGE("%s: No mem for enumeration, abort.", __FUNCTION__);
            goto abort;
        }
        memset(newDev, 0, sizeof(struct CameraSensor));

        newDev->devName = new char[PROPERTY_VALUE_MAX];
        if (!newDev->devName) {
            LOGE("%s: No mem for dev name, abort.", __FUNCTION__);
            goto abort;
        }

        // each camera device must have a name
        snprintf(propKey, sizeof(propKey), "%s.%d.%s", PROP_PREFIX, i, PROP_DEVNAME);
        if (0 == property_get(propKey, newDev->devName, 0)) {
            LOGE("%s: Failed to get name of camera %d from prop, abort.",
                    __FUNCTION__, i);
            goto abort;
        }

        // Setup facing info
        snprintf(propKey, sizeof(propKey), "%s.%d.%s", PROP_PREFIX, i, PROP_FACING);
        if (0 == property_get(propKey, propVal, 0)) {
            LOGE("%s: Failed to get facing of camera %d from prop, abort.",
                    __FUNCTION__, i);
            goto abort;
        }
        if (!strncmp(PROP_FACING_FRONT, propVal, strlen(PROP_FACING_FRONT))) {
            newDev->info.facing = CAMERA_FACING_FRONT;
        }
        else if (!strncmp(PROP_FACING_BACK, propVal, strlen(PROP_FACING_BACK))) {
            newDev->info.facing = CAMERA_FACING_BACK;
        }
        else {
            LOGE("%s: Invalid facing of camera %d from prop, abort.",
                    __FUNCTION__, i);
            goto abort;
        }

        // setup orientation
        snprintf(propKey, sizeof(propKey), "%s.%d.%s",
                PROP_PREFIX, i, PROP_ORIENTATION);

        if (0 == property_get(propKey, propVal, 0) ||
                (newDev->info.orientation = atoi(propVal)) < 0) {
            LOGE("%s: Invalid orientation of camera %d from prop, abort.",
                    __FUNCTION__, i);
            goto abort;
        }

        newDev->fd = -1;

        //It seems we get all info of a new camera
        LOGD("%s: Detected camera (%d) %s %s %d",
                __FUNCTION__, i, newDev->devName,
                newDev->info.facing == CAMERA_FACING_FRONT ? "front" : "back",
                newDev->info.orientation);
        mCameraSensor[i] = newDev;
        numCameras++;
    }

    return numCameras;

abort:
    LOGE("%s: Terminate camera enumeration !!", __FUNCTION__);
    cleanupCameras();
    //something wrong, further cleaning job
    if (newDev) {
        if (newDev->devName) {
            delete []newDev->devName;
            newDev->devName = 0;
        }
        delete newDev;
        newDev = 0;
    }
    return 0;
}

// Clean up camera  enumeration info
// Caller needs to take care syncing
void CameraDriver::cleanupCameras(){
    // clean up old enumeration
    LOG1("@%s: clean up", __FUNCTION__);
    for (int i = 0; i < MAX_CAMERAS; i++) {
        if (mCameraSensor[i]) {
            LOG1("@%s: found old camera (%d)", __FUNCTION__, i);
            struct CameraSensor *cam = mCameraSensor[i];
            if (cam->fd > 0) {
                // Should we release buffers?
                close(cam->fd);
                cam->fd = -1;
            }
            if (cam->devName) {
                delete []cam->devName;
                cam->devName = 0;
            }
            delete cam;
            mCameraSensor[i] = 0;
        }
    }
    numCameras = 0;
}

status_t CameraDriver::autoFocus()
{
    return INVALID_OPERATION;
}

status_t CameraDriver::cancelAutoFocus()
{
    return INVALID_OPERATION;
}

status_t CameraDriver::setEffect(Effect effect)
{
    if (effect != EFFECT_NONE) {
        LOGE("invalid color effect");
        return BAD_VALUE;
    }

    // Do nothing. EFFECT_NONE is all we support.

    return NO_ERROR;;
}

status_t CameraDriver::setFlashMode(FlashMode flashMode)
{
    if (flashMode != FLASH_MODE_OFF) {
        LOGE("invalid flash mode");
        return BAD_VALUE;
    }

    // Do nothing. FLASH_MODE_OFF is all we support.

    return NO_ERROR;;
}

status_t CameraDriver::setSceneMode(SceneMode sceneMode)
{
    if (sceneMode != SCENE_MODE_AUTO) {
        LOGE("invalid scene mode");
        return BAD_VALUE;
    }

    // Do nothing. SCENE_MODE_AUTO is all we support.

    return NO_ERROR;;
}

status_t CameraDriver::setFocusMode(FocusMode focusMode, CameraWindow *windows, int numWindows)
{
    if (focusMode != FOCUS_MODE_FIXED) {
        LOGE("invalid focus mode");
        return BAD_VALUE;
    }

    if (windows != NULL || numWindows != 0) {
        LOGE("focus windows not supported");
        return INVALID_OPERATION;
    }

    // Do nothing. FOCUS_MODE_FIXED is all we support.

    return NO_ERROR;
}

status_t CameraDriver::setWhiteBalanceMode(WhiteBalanceMode wbMode)
{
    if (wbMode != WHITE_BALANCE_AUTO) {
        LOGE("invalid white balance");
        return BAD_VALUE;
    }

    // Do nothing. WHITE_BALANCE_AUTO is all we support.

    return NO_ERROR;;
}

status_t CameraDriver::setAeLock(bool lock)
{
    LOGE("ae lock not supported");
    return INVALID_OPERATION;
}

status_t CameraDriver::setAwbLock(bool lock)
{
    LOGE("awb lock not supported");
    return INVALID_OPERATION;
}

status_t CameraDriver::setMeteringAreas(CameraWindow *windows, int numWindows)
{
    LOGE("metering not supported");
    return INVALID_OPERATION;
}

} // namespace android
