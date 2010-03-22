/*
**
** Copyright 2008, The Android Open Source Project
**
** Licensed under the Apache License, Version 2.0 (the "License");
** you may not use this file except in compliance with the License.
** You may obtain a copy of the License at
**
**     http://www.apache.org/licenses/LICENSE-2.0
**
** Unless required by applicable law or agreed to in writing, software
** distributed under the License is distributed on an "AS IS" BASIS,
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
** See the License for the specific language governing permissions and
** limitations under the License.
*/

#define LOG_TAG "CameraHardware"
#include <utils/Log.h>

#include "CameraHardware.h"
#include <utils/threads.h>
#include <fcntl.h>
#include <sys/mman.h>

namespace android {

CameraHardware::CameraHardware()
                  : mParameters(),
                    mHeap(0),
                    mRecordHeap(0),
                    mJpegHeap(0),
		    mRawHeap(0),
		    Camera(0),
		    cur_snr(0),
		    mPreviewRunning(false),
		    mRecordRunning(false),
                    mFrameSize(0),
                    mNotifyCb(0),
                    mDataCb(0),
                    mDataCbTimestamp(0),
                    mCallbackCookie(0),
                    mMsgEnabled(0),
                    mCurrentFrame(0)
{
    initDefaultParameters();
    Camera = new IntelCamera();
    cur_snr = Camera->get_sensor_infos();
    Camera->print_sensor_infos();

}

void CameraHardware::initDefaultParameters()
{
    CameraParameters p;

    p.setPreviewSize(176, 144);
    p.setPreviewFrameRate(15);
    p.setPreviewFormat("yuv422sp");
    p.setPictureSize(1600, 1200);
    p.setPictureFormat("jpeg");

    if (setParameters(p) != NO_ERROR) {
        LOGE("Failed to set default parameters?!");
    }
}

CameraHardware::~CameraHardware()
{
    delete Camera;
    singleton.clear();
}

sp<IMemoryHeap> CameraHardware::getPreviewHeap() const
{
    return mHeap;
}

sp<IMemoryHeap> CameraHardware::getRawHeap() const
{
    return mRawHeap;
}

void CameraHardware::setCallbacks(notify_callback notify_cb,
                                      data_callback data_cb,
                                      data_callback_timestamp data_cb_timestamp,
                                      void* user)
{
    Mutex::Autolock lock(mLock);
    mNotifyCb = notify_cb;
    mDataCb = data_cb;
    mDataCbTimestamp = data_cb_timestamp;
    mCallbackCookie = user;
}

void CameraHardware::enableMsgType(int32_t msgType)
{
    Mutex::Autolock lock(mLock);
    mMsgEnabled |= msgType;
}

void CameraHardware::disableMsgType(int32_t msgType)
{
    Mutex::Autolock lock(mLock);
    mMsgEnabled &= ~msgType;
}

bool CameraHardware::msgTypeEnabled(int32_t msgType)
{
    Mutex::Autolock lock(mLock);
    return (mMsgEnabled & msgType);
}

// ---------------------------------------------------------------------------

int CameraHardware::previewThread()
{
    // Notify the client of a new frame.
    if (mPreviewRunning) {
        if ( mMsgEnabled & CAMERA_MSG_PREVIEW_FRAME) {
	    mCurrentFrame = Camera->capture_grab_frame(mHeap->getBase());
	    if (mMsgEnabled & CAMERA_MSG_VIDEO_FRAME) {
	        if (mRecordBufferStatus) {
		    mRecordBufferStatus = false;
	            nsecs_t timeStamp = systemTime(SYSTEM_TIME_MONOTONIC);
		    mCurrentFrame = Camera->capture_grab_record_frame(mRecordHeap->getBase());
		    mDataCbTimestamp(timeStamp, CAMERA_MSG_VIDEO_FRAME, mRecordBuffer, mCallbackCookie);
		    LOGD("%s : Recording mCurrentFrame = %u", __func__, mCurrentFrame);
		}
	    }
	    LOGD("%s : mCurrentFrame = %u", __func__, mCurrentFrame);
	    mDataCb(CAMERA_MSG_PREVIEW_FRAME, mBuffer, mCallbackCookie);
	    Camera->capture_recycle_frame();
	    usleep(mDelay);
	}
    }
    return NO_ERROR;
}

status_t CameraHardware::startPreview()
{
    Mutex::Autolock lock(mLock);
    if (mPreviewThread != 0) {
        // already running
        return INVALID_OPERATION;
    }
    int w, h;
    mParameters.getPreviewSize(&w, &h);
    mFrameSize = w * h * 2;
    mHeap = new MemoryHeapBase(mFrameSize);
    mBuffer = new MemoryBase(mHeap, 0, mFrameSize);
    int fps = mParameters.getPreviewFrameRate();
    mDelay = (int)(1000000.0f / float(fps));
    LOGD("%s w=%d, h=%d, mFrameSize=%d, fps=%d",__func__,w,h,mFrameSize,fps);
    //    Camera->capture_init(w, h, INTEL_PIX_FMT_YUYV, 3);
    Camera->capture_init(w, h, INTEL_PIX_FMT_NV12, 3);
    Camera->capture_start();
    Camera->capture_map_frame();

    Camera->capture_setup_AE(ON);
    Camera->capture_setup_AWB(ON);
    Camera->capture_setup_AF(OFF);
    Camera->capture_setup_image_effect(CI_IE_MODE_OFF);

    mPreviewThread = new PreviewThread(this);
    mPreviewRunning = true;

    return NO_ERROR;
}

void CameraHardware::stopPreview()
{
    sp<PreviewThread> previewThread;
    { // scope for the lock
        Mutex::Autolock lock(mLock);
        previewThread = mPreviewThread;
    }

    // don't hold the lock while waiting for the thread to quit
    if (previewThread != 0) {
      previewThread->requestExitAndWait();
    }

    Mutex::Autolock lock(mLock);
    mPreviewThread.clear();

    if (mPreviewRunning) {
      Camera->capture_unmap_frame();
      Camera->capture_finalize();
    }

    mPreviewRunning = false;
}

bool CameraHardware::previewEnabled() {
    return mPreviewRunning;
}

status_t CameraHardware::startRecording()
{
    int w, h, framesize;
    mParameters.getPreviewSize(&w, &h);
    framesize = w * h * 2;
    mRecordHeap = new MemoryHeapBase(framesize);
    mRecordBuffer = new MemoryBase(mRecordHeap, 0, framesize);

    LOGD("%s w=%d, h=%d, mFrameSize=%d, mRecordHeap->getBase()=%p",
	 __func__,w,h,framesize,mRecordHeap->getBase());

    mRecordRunning = true;
    return NO_ERROR;
}

void CameraHardware::stopRecording()
{
    mRecordRunning = false;
}

bool CameraHardware::recordingEnabled()
{
    return mRecordRunning;
}

void CameraHardware::releaseRecordingFrame(const sp<IMemory>& mem)
{
  //    Mutex::Autolock lock(mLock);
    mRecordBufferStatus = true;
    LOGD("%s: mem->pointer() = %p, mem->size() = %d, mem->offset() = %d",
	 __func__,mem->pointer(), (int)mem->size(), (int)mem->offset());
}

// ---------------------------------------------------------------------------

int CameraHardware::beginAutoFocusThread(void *cookie)
{
    CameraHardware *c = (CameraHardware *)cookie;
    return c->autoFocusThread();
}

int CameraHardware::autoFocusThread()
{
    if (mMsgEnabled & CAMERA_MSG_FOCUS) {
        Camera->auto_focus_process();
	Camera->auto_exposure_process();
	Camera->auto_white_balance_process();
	mNotifyCb(CAMERA_MSG_FOCUS, true, 0, mCallbackCookie);
    }
    return NO_ERROR;
}

status_t CameraHardware::autoFocus()
{
    Mutex::Autolock lock(mLock);
    if (createThread(beginAutoFocusThread, this) == false)
        return UNKNOWN_ERROR;
    return NO_ERROR;
}

status_t CameraHardware::cancelAutoFocus()
{
    return NO_ERROR;
}

/*static*/ int CameraHardware::beginPictureThread(void *cookie)
{
    CameraHardware *c = (CameraHardware *)cookie;
    return c->pictureThread();
}

int CameraHardware::pictureThread()
{
    if (mMsgEnabled & CAMERA_MSG_SHUTTER)
        mNotifyCb(CAMERA_MSG_SHUTTER, 0, 0, mCallbackCookie);

    if (mMsgEnabled & CAMERA_MSG_RAW_IMAGE) {
        //FIXME: use a canned YUV image!
        // In the meantime just make another fake camera picture.
      //          mDataCb(CAMERA_MSG_RAW_IMAGE, mBuffer, mCallbackCookie);
    }
    if (mMsgEnabled & CAMERA_MSG_COMPRESSED_IMAGE) {
      int w,h;
      mParameters.getPictureSize(&w, &h);
      mFrameSize = w * h * 2;
      mJpegHeap = new MemoryHeapBase(mFrameSize);
      mJpegBuffer = new MemoryBase(mJpegHeap, 0, mFrameSize);
      LOGD("%s w=%d, h=%d, mFrameSize=%d",__func__,w,h,mFrameSize);

      Camera->capture_init(w, h, INTEL_PIX_FMT_JPEG, 1);
      Camera->capture_start();
      Camera->capture_map_frame();

      Camera->capture_setup_AE(ON);
      Camera->capture_setup_AWB(ON);
      Camera->capture_setup_AF(OFF);
      Camera->capture_setup_image_effect(CI_IE_MODE_OFF);

      mCurrentFrame = Camera->capture_grab_frame(mJpegHeap->getBase());
      mDataCb(CAMERA_MSG_COMPRESSED_IMAGE, mJpegBuffer, mCallbackCookie);
      Camera->capture_unmap_frame();
      Camera->capture_recycle_frame();

      Camera->capture_finalize();
    }
    return NO_ERROR;
}

status_t CameraHardware::takePicture()
{
    stopPreview();
    if (createThread(beginPictureThread, this) == false)
        return -1;

    return NO_ERROR;
}

status_t CameraHardware::cancelPicture()
{
    return NO_ERROR;
}

status_t CameraHardware::dump(int fd, const Vector<String16>& args) const
{
    return NO_ERROR;
}

status_t CameraHardware::setParameters(const CameraParameters& params)
{
    Mutex::Autolock lock(mLock);
    // XXX verify params

    if (strcmp(params.getPreviewFormat(), "yuv422sp") != 0) {
        LOGE("Only yuv422sp preview is supported");
        return -1;
    }

    if (strcmp(params.getPictureFormat(), "jpeg") != 0) {
        LOGE("Only jpeg still pictures are supported");
        return -1;
    }

    int w,h,fps;
    mParameters = params;

    params.getPreviewSize(&w,&h);
    if( (w != 640) && (h != 480) ) {
      w = 640;
      h = 480;
    }

    mParameters.setPreviewSize(w,h);
    LOGD("PREVIEW SIZE: w=%d h=%d", w, h);

    fps = params.getPreviewFrameRate();
    mParameters.setPreviewFrameRate(fps);
    LOGD("PICTURE FPS: %d",fps);

    params.getPictureSize(&w, &h);
    if(Camera != NULL) {
      if (Camera->is_sensor_support_resolution(w,h) == 0) {
	LOGE("the resolution w=%d * h=%d are not supported",w,h);
	Camera->get_max_sensor_resolution(&w, &h);
	LOGD("suitable resolution w=%d * h=%d",w,h);
      }
    }

    mParameters.setPictureSize(w, h);
    LOGD("PICTURE SIZE: w=%d h=%d", w, h);

    /*
    LOGI("Gets the current color effect setting =\n\t %s",params.getColorEffect());
    LOGI("Gets the current flash mode setting =\n\t %s", params.getFlashMode());
    LOGI("gets the current focus mode setting =\n\t %s", params.getFocusMode());
    LOGI("Returns the quality setting for the JPEG picture =\n\t %d", params.getJpegQuality());
    LOGI("Returns the image format for pictures =\n\t %d", params.getPictureFormat());
    LOGI("Returns the image format for preview pictures got from Camera.PreviewCallback =\n\t %d", params.getPreviewFormat());
    LOGI("Returns the setting for the rate at which preview frames are received =\n\t %d", params.getPreviewFrameRate());
    */
    return NO_ERROR;

}

CameraParameters CameraHardware::getParameters() const
{
    Mutex::Autolock lock(mLock);
    return mParameters;
}

status_t CameraHardware::sendCommand(int32_t command, int32_t arg1,
                                         int32_t arg2)
{
    return BAD_VALUE;
}

void CameraHardware::release()
{
}

wp<CameraHardwareInterface> CameraHardware::singleton;

sp<CameraHardwareInterface> CameraHardware::createInstance()
{
    if (singleton != 0) {
        sp<CameraHardwareInterface> hardware = singleton.promote();
        if (hardware != 0) {
            return hardware;
        }
    }
    sp<CameraHardwareInterface> hardware(new CameraHardware());
    singleton = hardware;
    return hardware;
}

extern "C" sp<CameraHardwareInterface> openCameraHardware()
{
    return CameraHardware::createInstance();
}

}; // namespace android
