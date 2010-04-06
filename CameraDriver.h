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

#ifndef ANDROID_LIBCAMERA_CAMERA_DRIVER
#define ANDROID_LIBCAMERA_CAMERA_DRIVER

#include <utils/Timers.h>
#include <utils/Errors.h>
#include <utils/Vector.h>
#include <utils/Errors.h>
#include <utils/threads.h>
#include <camera/CameraParameters.h>
#include "CameraCommon.h"

namespace android {

class Callbacks;

class CameraDriver {

// public types
public:

// constructor/destructor
public:
    CameraDriver(int cameraId);
    ~CameraDriver();

// public types
    enum Mode {
        MODE_NONE,
        MODE_PREVIEW,
        MODE_CAPTURE,
        MODE_VIDEO,
    };

    enum Effect {
        EFFECT_NONE,
        EFFECT_MONO,
        EFFECT_NEGATIVE,
        EFFECT_SOLARIZE,
        EFFECT_SEPIA,
        EFFECT_POSTERIZE,
        EFFECT_WHITEBOARD,
        EFFECT_BLACKBOARD,
        EFFECT_AQUA,
    };

    enum FlashMode {
        FLASH_MODE_OFF,
        FLASH_MODE_AUTO,
        FLASH_MODE_ON,
        FLASH_MODE_RED_EYE,
        FLASH_MODE_TORCH,
    };

    enum SceneMode {
        SCENE_MODE_AUTO,
        SCENE_MODE_ACTION,
        SCENE_MODE_PORTRAIT,
        SCENE_MODE_LANDSCAPE,
        SCENE_MODE_NIGHT,
        SCENE_MODE_NIGHT_PORTRAIT,
        SCENE_MODE_THEATRE,
        SCENE_MODE_BEACH,
        SCENE_MODE_SNOW,
        SCENE_MODE_SUNSET,
        SCENE_MODE_STEADYPHOTO,
        SCENE_MODE_FIREWORKS,
        SCENE_MODE_SPORTS,
        SCENE_MODE_PARTY,
        SCENE_MODE_CANDLELIGHT,
        SCENE_MODE_BARCODE,
    };

    enum FocusMode {
        FOCUS_DISTANCE_INFINITY,
        FOCUS_MODE_AUTO,
        FOCUS_MODE_INFINITY,
        FOCUS_MODE_MACRO,
        FOCUS_MODE_FIXED,
        FOCUS_MODE_EDOF,
        FOCUS_MODE_CONTINUOUS_VIDEO,
        FOCUS_MODE_CONTINUOUS_PICTURE,
    };

    enum WhiteBalanceMode {
        WHITE_BALANCE_AUTO,
        WHITE_BALANCE_INCANDESCENT,
        WHITE_BALANCE_FLUORESCENT,
        WHITE_BALANCE_WARM_FLUORESCENT,
        WHITE_BALANCE_DAYLIGHT,
        WHITE_BALANCE_CLOUDY_DAYLIGHT,
        WHITE_BALANCE_TWILIGHT,
        WHITE_BALANCE_SHADE,
    };

// public methods
public:

    void getDefaultParameters(CameraParameters *params);

    status_t start(Mode mode);
    status_t stop();

    inline int getNumBuffers() { return mNumBuffers; }

    status_t getPreviewFrame(CameraBuffer *buff);
    status_t putPreviewFrame(CameraBuffer *buff);

    status_t getRecordingFrame(CameraBuffer *buff, nsecs_t *timestamp);
    status_t putRecordingFrame(CameraBuffer *buff);

    status_t setSnapshotBuffers(void *buffs, int numBuffs);
    status_t getSnapshot(CameraBuffer *buff);
    status_t putSnapshot(CameraBuffer *buff);

    status_t getThumbnail(CameraBuffer *buff);
    status_t putThumbnail(CameraBuffer *buff);

    bool dataAvailable();
    bool isBufferValid(const CameraBuffer * buffer) const;

    status_t setPreviewFrameFormat(int width, int height, int format = 0);
    status_t setPostviewFrameFormat(int width, int height, int format);
    status_t setSnapshotFrameFormat(int width, int height, int format);
    status_t setVideoFrameFormat(int width, int height, int format = 0);

    inline int getSnapshotPixelFormat() { return mConfig.snapshot.format; }
    void getVideoSize(int *width, int *height);

    void getZoomRatios(Mode mode, CameraParameters *params);
    void getFocusDistances(CameraParameters *params);
    status_t setZoom(int zoom);

    // EXIF params
    status_t getFNumber(unsigned int *fNumber); // format is: (numerator << 16) | denominator
    status_t getExposureInfo(CamExifExposureProgramType *exposureProgram,
                             CamExifExposureModeType *exposureMode,
                             int *exposureTime,
                             float *exposureBias,
                             int *aperture);
    status_t getBrightness(float *brightness);
    status_t getIsoSpeed(int *isoSpeed);
    status_t getMeteringMode(CamExifMeteringModeType *meteringMode);
    status_t getAWBMode(CamExifWhiteBalanceType *wbMode);
    status_t getSceneMode(CamExifSceneCaptureType *sceneMode);

    // camera hardware information
    static int getNumberOfCameras();
    static status_t getCameraInfo(int cameraId, camera_info *cameraInfo);

    float getFrameRate() { return mConfig.fps; }

    status_t autoFocus();
    status_t cancelAutoFocus();

    status_t setEffect(Effect effect);
    status_t setFlashMode(FlashMode flashMode);
    status_t setSceneMode(SceneMode sceneMode);
    status_t setFocusMode(FocusMode focusModei,
                          CameraWindow *windows = 0,    // optional
                          int numWindows = 0);          // optional
    status_t setWhiteBalanceMode(WhiteBalanceMode wbMode);
    status_t setAeLock(bool lock);
    status_t setAwbLock(bool lock);
    status_t setMeteringAreas(CameraWindow *windows, int numWindows);

// private types
private:

    static const int MAX_DRIVER_BUFFERS  = MAX_BURST_BUFFERS;
    static const int MAX_CAMERAS         = 8;
    static const int NUM_DEFAULT_BUFFERS = 4;

    struct FrameInfo {
        int format;     // V4L2 format
        int width;      // Frame width
        int height;     // Frame height
        int padding;    // Frame padding width
        int maxWidth;   // Frame maximum width
        int maxHeight;  // Frame maximum height
        int size;       // Frame size in bytes
    };

    struct Config {
        FrameInfo preview;    // preview
        FrameInfo recording;  // recording
        FrameInfo snapshot;   // snapshot
        FrameInfo postview;   // postview (thumbnail for capture)
        float fps;            // preview/recording (shared)
        int num_snapshot;     // number of snapshots to take
        int zoom;             // zoom value
    };

    struct CameraSensor {
        char *devName;              // device node's name, e.g. /dev/video0
        struct camera_info info;    // camera info defined by Android
        int fd;                     // the file descriptor of device at run time

        /* more fields will be added when we find more 'per camera' data*/
    };

    struct DriverBuffer {
        void *data;
        size_t length;
        int width;
        int height;
        int format;
        int flags; //You can use to to detern the buf status
        struct v4l2_buffer vbuffer;
    };

    struct DriverBufferPool {
        int active_buffers;
        int width;
        int height;
        int format;
        struct DriverBuffer bufs [MAX_DRIVER_BUFFERS];
    };

// private methods
private:

    status_t startPreview();
    status_t stopPreview();
    status_t startRecording();
    status_t stopRecording();
    status_t startCapture();
    status_t stopCapture();

    status_t allocatePreviewBuffers();
    status_t allocateRecordingBuffers();
    status_t allocateSnapshotBuffers();
    status_t freePreviewBuffers();
    status_t freeRecordingBuffers();
    status_t freeSnapshotBuffers();
    static int enumerateCameras();
    static void cleanupCameras();
    const char* getMaxSnapShotResolution();

    int openDevice();
    void closeDevice();
    int configureDevice(Mode deviceMode, int w, int h, int format, bool raw);
    int startDevice(int buffer_count);
    void stopDevice();

    status_t v4l2_capture_open(const char *devName);
    status_t v4l2_capture_close(int fd);
    status_t v4l2_capture_querycap(int fd, struct v4l2_capability *cap);
    status_t v4l2_capture_s_input(int fd, int index);
    int detectDeviceResolutions();
    int set_capture_mode(Mode deviceMode);
    int v4l2_capture_try_format(int fd, int *w, int *h, int *format);
    int v4l2_capture_g_framerate(int fd, float * framerate, int width,
                                          int height, int pix_fmt);
    int v4l2_capture_s_format(int fd, int w, int h, int format, bool raw);
    int v4l2_capture_streamoff(int fd);
    void destroyBufferPool();
    int v4l2_capture_free_buffer(struct DriverBuffer *buf_info);
    int v4l2_capture_release_buffers();
    int v4l2_capture_request_buffers(uint num_buffers);
    int createBufferPool(int buffer_count);
    int v4l2_capture_new_buffer(int index, struct DriverBuffer *buf);
    int activateBufferPool();
    int v4l2_capture_streamon(int fd);
    int v4l2_capture_qbuf(int fd, int index, struct DriverBuffer *buf);
    int grabFrame(struct v4l2_buffer *buf);
    int v4l2_capture_dqbuf(int fd, struct v4l2_buffer *buf);
    int set_attribute (int fd, int attribute_num,
                               const int value, const char *name);
    int set_zoom (int fd, int zoom);
    int xioctl(int fd, int request, void *arg);

    // private members
private:

    static int numCameras;
    static Mutex mCameraSensorLock;                             // lock to access mCameraSensor
    static struct CameraSensor *mCameraSensor[MAX_CAMERAS];     // all camera sensors in CameraDriver Class.

    Mode mMode;
    Callbacks *mCallbacks;

    int mNumBuffers;
    CameraBuffer *mPreviewBuffers;
    CameraBuffer *mRecordingBuffers;

    CameraBuffer mSnapshotBuffers[MAX_BURST_BUFFERS];
    CameraBuffer mPostviewBuffers[MAX_BURST_BUFFERS];
    int mNumPreviewBuffersQueued;
    int mNumRecordingBuffersQueued;
    int mNumCapturegBuffersQueued;
    Config mConfig;

    struct DriverBufferPool mBufferPool[MAX_CAMERAS]; // FIXME: Number of buffers should be dynamically queried.

    int mSessionId; // uniquely identify each session

    int mCameraId;

}; // class CameraDriver

}; // namespace android

#endif // ANDROID_LIBCAMERA_CAMERA_DRIVER
