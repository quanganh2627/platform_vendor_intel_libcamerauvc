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
#define LOG_TAG "Camera_ControlThread"

#include "ControlThread.h"
#include "LogHelper.h"
#include "PreviewThread.h"
#include "PictureThread.h"
#include "CameraDriver.h"
#include "Callbacks.h"
#include "ColorConverter.h"
#include "FaceDetectorFactory.h"
#include "EXIFFields.h"
#include <utils/Vector.h>
#include <math.h>

namespace android {

/*
 * ASPECT_TOLERANCE: the tolerance between aspect ratios to consider them the same
 */
#define ASPECT_TOLERANCE 0.001

ControlThread::ControlThread(int cameraId) :
    Thread(true) // callbacks may call into java
    ,mDriver(new CameraDriver(cameraId))
    ,mPreviewThread(new PreviewThread())
    ,mPictureThread(new PictureThread())
    ,mVideoThread(new VideoThread())
    ,mPipeThread(new PipeThread())
    ,mMessageQueue("ControlThread", (int) MESSAGE_ID_MAX)
    ,mState(STATE_STOPPED)
    ,mThreadRunning(false)
    ,mCallbacks(Callbacks::getInstance())
    ,mNumBuffers(mDriver->getNumBuffers())
    ,m_pFaceDetector(0)
    ,mFaceDetectionActive(false)
    ,mThumbSupported(false)
    ,mLastRecordingBuff(0)
    ,mCameraFormat(mDriver->getFormat())
{
    LOG1("@%s: cameraId = %d", __FUNCTION__, cameraId);

    initDefaultParams();

    mPipeThread->setThreads(mPreviewThread, mVideoThread);

    status_t status = mPreviewThread->run();
    if (status != NO_ERROR) {
        ALOGE("Error starting preview thread!");
    }
    status = mPictureThread->run();
    if (status != NO_ERROR) {
        ALOGW("Error starting picture thread!");
    }
    status = mVideoThread->run();
    if (status != NO_ERROR) {
        ALOGW("Error starting video thread!");
    }
    status = mPipeThread->run();
    if (status != NO_ERROR) {
        ALOGW("Error starting pipe thread!");
    }
    m_pFaceDetector=FaceDetectorFactory::createDetector(mCallbacks);
    if (m_pFaceDetector != 0){
        mParameters.set(CameraParameters::KEY_MAX_NUM_DETECTED_FACES_HW,
                m_pFaceDetector->getMaxFacesDetectable());
    } else {
        ALOGE("Failed on creating face detector.");
    }
}

ControlThread::~ControlThread()
{
    LOG1("@%s", __FUNCTION__);

    mPreviewThread->requestExitAndWait();
    mPreviewThread.clear();

    mPictureThread->requestExitAndWait();
    mPictureThread.clear();

    mVideoThread->requestExitAndWait();
    mVideoThread.clear();

    mPipeThread->requestExitAndWait();
    mPipeThread.clear();

    if (mDriver != NULL) {
        delete mDriver;
    }
    if (mCallbacks != NULL) {
        delete mCallbacks;
    }
    if (m_pFaceDetector != 0) {
        if (!FaceDetectorFactory::destroyDetector(m_pFaceDetector)){
            ALOGE("Failed on destroy face detector thru factory");
            delete m_pFaceDetector;//should not happen.
        }
        m_pFaceDetector = 0;
    }
}

void ControlThread::initDefaultParams()
{
    // get default params from CameraDriver and JPEG encoder
    mDriver->getDefaultParameters(&mParameters);
    mPictureThread->getDefaultParameters(&mParameters);

    // preview format
    mParameters.setPreviewFormat(CameraParameters::PIXEL_FORMAT_YUV420SP);

    // supported preview formats
    char previewFormats[100] = {0};
    snprintf(previewFormats, sizeof(previewFormats), "%s",
            CameraParameters::PIXEL_FORMAT_YUV420SP);

    // video format
    mParameters.set(CameraParameters::KEY_VIDEO_FRAME_FORMAT,
            CameraParameters::PIXEL_FORMAT_YUV420SP);
}

status_t ControlThread::setPreviewWindow(struct preview_stream_ops *window)
{
    LOG1("@%s: window = %p", __FUNCTION__, window);
    if (mPreviewThread != NULL) {
        return mPreviewThread->setPreviewWindow(window);
    }
    return NO_ERROR;
}

void ControlThread::setCallbacks(camera_notify_callback notify_cb,
                                 camera_data_callback data_cb,
                                 camera_data_timestamp_callback data_cb_timestamp,
                                 camera_request_memory get_memory,
                                 void* user)
{
    LOG1("@%s", __FUNCTION__);
    mCallbacks->setCallbacks(notify_cb,
            data_cb,
            data_cb_timestamp,
            get_memory,
            user);
}

void ControlThread::enableMsgType(int32_t msgType)
{
    LOG2("@%s", __FUNCTION__);
    mCallbacks->enableMsgType(msgType);
}

void ControlThread::disableMsgType(int32_t msgType)
{
    LOG2("@%s", __FUNCTION__);
    mCallbacks->disableMsgType(msgType);
}

bool ControlThread::msgTypeEnabled(int32_t msgType)
{
    LOG2("@%s", __FUNCTION__);
    return mCallbacks->msgTypeEnabled(msgType);
}

status_t ControlThread::startPreview()
{
    LOG1("@%s", __FUNCTION__);
    // send message and block until thread processes message
    Message msg;
    msg.id = MESSAGE_ID_START_PREVIEW;
    return mMessageQueue.send(&msg, MESSAGE_ID_START_PREVIEW);
}

status_t ControlThread::stopPreview()
{
    LOG1("@%s", __FUNCTION__);
    // send message and block until thread processes message
    if (mState == STATE_STOPPED) {
        return NO_ERROR;
    }

    Message msg;
    msg.id = MESSAGE_ID_STOP_PREVIEW;
    return mMessageQueue.send(&msg, MESSAGE_ID_STOP_PREVIEW);
}

status_t ControlThread::startRecording()
{
    LOG1("@%s", __FUNCTION__);
    // send message and block until thread processes message
    Message msg;
    msg.id = MESSAGE_ID_START_RECORDING;
    return mMessageQueue.send(&msg, MESSAGE_ID_START_RECORDING);
}

status_t ControlThread::stopRecording()
{
    LOG1("@%s", __FUNCTION__);
    // send message and block until thread processes message
    Message msg;
    msg.id = MESSAGE_ID_STOP_RECORDING;
    return mMessageQueue.send(&msg, MESSAGE_ID_STOP_RECORDING);
}

bool ControlThread::previewEnabled()
{
    LOG2("@%s", __FUNCTION__);
    bool enabled = (mState == STATE_PREVIEW_STILL ||
            mState == STATE_PREVIEW_VIDEO ||
            mState == STATE_RECORDING);
    return enabled;
}

bool ControlThread::recordingEnabled()
{
    LOG2("@%s", __FUNCTION__);
    return mState == STATE_RECORDING;
}

status_t ControlThread::setParameters(const char *params)
{
    LOG1("@%s: params = %p", __FUNCTION__, params);
    Message msg;
    msg.id = MESSAGE_ID_SET_PARAMETERS;
    msg.data.setParameters.params = const_cast<char*>(params); // We swear we won't modify params :)
    return mMessageQueue.send(&msg, MESSAGE_ID_SET_PARAMETERS);
}

char* ControlThread::getParameters()
{
    LOG1("@%s", __FUNCTION__);

    char *params = NULL;
    Message msg;
    msg.id = MESSAGE_ID_GET_PARAMETERS;
    msg.data.getParameters.params = &params; // let control thread allocate and set pointer
    mMessageQueue.send(&msg, MESSAGE_ID_GET_PARAMETERS);
    return params;
}

void ControlThread::putParameters(char* params)
{
    LOG1("@%s: params = %p", __FUNCTION__, params);
    if (params)
        free(params);
}

bool ControlThread::isParameterSet(const char* param)
{
    const char* strParam = mParameters.get(param);
    int len = strlen(CameraParameters::TRUE);
    if (strParam != NULL && strncmp(strParam, CameraParameters::TRUE, len) == 0) {
        return true;
    }
    return false;
}

bool ControlThread::isThumbSupported(State state)
{
    bool supported = false;

    // thumbnail is supported if width and height are non-zero
    // and shot is snapped in still picture mode. thumbnail is
    // not supported for video snapshot.
    if (state == STATE_PREVIEW_STILL || state == STATE_CAPTURE) {
        int width = mParameters.getInt(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH);
        int height = mParameters.getInt(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT);
        supported = (width != 0) && (height != 0);
    }

    return supported;
}

status_t ControlThread::gatherExifInfo(const CameraParameters *params, bool flash, exif_attribute_t *exif)
{
    status_t status = NO_ERROR;
    if (params == NULL || exif == NULL) {
        ALOGE("null params in %s", __FUNCTION__);
        return BAD_VALUE;
    }

    EXIFFields fields;
    //
    // GENERAL DATA
    //
    int pictureWidth;
    int pictureHeight;
    int thumbnailWidth;
    int thumbnailHeight;
    params->getPictureSize(&pictureWidth, &pictureHeight);
    thumbnailWidth = params->getInt(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH);
    thumbnailHeight = params->getInt(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT);

    int rotation = params->getInt(CameraParameters::KEY_ROTATION);
    ExifOrientationType orientation;
    switch (rotation) {
    case 0:
        orientation = EXIF_ORIENTATION_UP;
        break;
    case 90:
        orientation = EXIF_ORIENTATION_90;
        break;
    case 180:
        orientation = EXIF_ORIENTATION_180;
        break;
    case 270:
        orientation = EXIF_ORIENTATION_270;
        break;
    default:
        orientation = EXIF_ORIENTATION_UP;
        break;
    };

    fields.setGeneralFields(flash,
            pictureWidth,
            pictureHeight,
            thumbnailWidth,
            thumbnailHeight,
            orientation);

    //
    // GPS DATA
    //
    const char *pLatitude = params->get(CameraParameters::KEY_GPS_LATITUDE);
    const char *pLongitude = params->get(CameraParameters::KEY_GPS_LONGITUDE);
    const char *pAltitude = params->get(CameraParameters::KEY_GPS_ALTITUDE);
    const char *pTimestamp = params->get(CameraParameters::KEY_GPS_TIMESTAMP);
    const char *pProcessingMethod = params->get(CameraParameters::KEY_GPS_PROCESSING_METHOD);

    if (pLatitude && pLongitude) { // latitude and longitude are required
        float latitude = 0.0;
        float longitude = 0.0;
        float altitude = 0.0;
        long timestamp = 0;

        latitude = atof(pLatitude);
        longitude = atof(pLongitude);

        if (pAltitude)
            altitude = atof(pAltitude);

        if (pTimestamp)
            timestamp = atol(pTimestamp);

        fields.setGPSFields(timestamp,
                latitude,
                longitude,
                altitude,
                pProcessingMethod);
    }

    //
    // HARDWARE DATA
    //
    unsigned int fNumber;
    float focalLength;

    CamExifExposureProgramType exposureProgram;
    CamExifExposureModeType exposureMode;
    int exposureTime;
    float exposureBias;
    int aperture;

    status = mDriver->getFNumber(&fNumber);
    if (status != NO_ERROR) {
        ALOGE("failed to get fNumber");
        return status;
    }

    focalLength = params->getFloat(CameraParameters::KEY_FOCAL_LENGTH);

    status = mDriver->getExposureInfo(&exposureProgram,
                             &exposureMode,
                             &exposureTime,
                             &exposureBias,
                             &aperture);
    if (status != NO_ERROR) {
        ALOGE("failed to get exposure info");
        return status;
    }

    float brightness;
    int isoSpeed;

    status = mDriver->getBrightness(&brightness);
    if (status != NO_ERROR) {
        ALOGE("failed to get brightness");
        return status;
    }

    status = mDriver->getIsoSpeed(&isoSpeed);
    if (status != NO_ERROR) {
        ALOGE("failed to get iso speed");
        return status;
    }

    CamExifMeteringModeType meteringMode;
    CamExifWhiteBalanceType wbMode;
    CamExifSceneCaptureType sceneMode;

    status = mDriver->getMeteringMode(&meteringMode);
    if (status != NO_ERROR) {
        ALOGE("failed to get metering mode");
        return status;
    }

    status = mDriver->getAWBMode(&wbMode);
    if (status != NO_ERROR) {
        ALOGE("failed to get awb mode");
        return status;
    }

    status = mDriver->getSceneMode(&sceneMode);
    if (status != NO_ERROR) {
        ALOGE("failed to get scene mode");
        return status;
    }

    fields.setHardwareFields(focalLength,
            fNumber,
            exposureProgram,
            exposureMode,
            exposureTime,
            aperture,
            brightness,
            exposureBias,
            isoSpeed,
            meteringMode,
            wbMode,
            sceneMode);

    fields.combineFields(exif);

    return status;
}

status_t ControlThread::takePicture()
{
    LOG1("@%s", __FUNCTION__);
    Message msg;
    msg.id = MESSAGE_ID_TAKE_PICTURE;
    return mMessageQueue.send(&msg);
}

status_t ControlThread::cancelPicture()
{
    LOG1("@%s", __FUNCTION__);
    Message msg;
    msg.id = MESSAGE_ID_CANCEL_PICTURE;
    return mMessageQueue.send(&msg);
}

status_t ControlThread::autoFocus()
{
    LOG1("@%s", __FUNCTION__);
    Message msg;
    msg.id = MESSAGE_ID_AUTO_FOCUS;
    return mMessageQueue.send(&msg);
}

status_t ControlThread::cancelAutoFocus()
{
    LOG1("@%s", __FUNCTION__);
    Message msg;
    msg.id = MESSAGE_ID_CANCEL_AUTO_FOCUS;
    return mMessageQueue.send(&msg);
}

status_t ControlThread::releaseRecordingFrame(void *buff)
{
    LOG2("@%s: buff = %p", __FUNCTION__, buff);
    Message msg;
    msg.id = MESSAGE_ID_RELEASE_RECORDING_FRAME;
    msg.data.releaseRecordingFrame.buff = buff;
    return mMessageQueue.send(&msg);
}

void ControlThread::returnBuffer(CameraBuffer *buff)
{
    Message msg;
    msg.id = MESSAGE_ID_RETURN_BUFFER;
    msg.data.returnBuffer.buff = buff;

    mMessageQueue.send(&msg);
}

status_t ControlThread::returnPreviewBuffer(CameraBuffer *buff)
{
    status_t status = NO_ERROR;
    status = mDriver->putPreviewFrame(buff);
    if (status != NO_ERROR) {
        ALOGE("Error putting preview frame to driver");
    }

    buff->mOwner = 0;//force reset the owner
    return status;
}

status_t ControlThread::returnVideoBuffer(CameraBuffer *buff)
{
    status_t status = NO_ERROR;
    status = mDriver->putRecordingFrame(buff);
    if (status == DEAD_OBJECT) {
        LOG1("Stale recording buffer returned to driver");
    } else if (status != NO_ERROR) {
        ALOGE("Error putting recording frame to driver");
    }
    buff->mOwner = 0;
    return status;
}

status_t ControlThread::returnSnapshotBuffer(CameraBuffer *buff)
{
    status_t status = NO_ERROR;
    status = mDriver->putSnapshot(buff);
    if (status != NO_ERROR) {
        ALOGE("Error in putting snapshot!");
        return status;
    }
    buff->mOwner = 0;
    return status;
}

status_t ControlThread::returnThumbnailBuffer(CameraBuffer *buff)
{
    status_t status = NO_ERROR;
    status = mDriver->putThumbnail(buff);
    if (status != NO_ERROR) {
        ALOGE("Error in putting thumbnail!");
        return status;
    }
    buff->mOwner = 0;
    return status;
}

status_t ControlThread::returnConversionBuffer(CameraBuffer *buff)
{
    status_t status = NO_ERROR;

    if (mConversionBuffers == 0)
        return status;

    for (int i = 0; i < mNumBuffers; i++) {
        if (&mConversionBuffers[i] == buff && buff->mID ==i) {
            mFreeBuffers.push_front(buff);
            return status;
        }
    }
    return DEAD_OBJECT;
}

int ControlThread::sendCommand(int32_t cmd, int32_t arg1, int32_t arg2)
{
    Message msg;
    int status = NO_ERROR;
    msg.id = MESSAGE_ID_COMMAND;
    msg.data.command.cmd_id = cmd;
    msg.data.command.arg1 = arg1;
    msg.data.command.arg2 = arg2;
    if(cmd == CAMERA_CMD_START_FACE_DETECTION){
        if(m_pFaceDetector == 0){
           status = BAD_VALUE;
        }
    }
    mMessageQueue.send(&msg);
    return status;
}

void ControlThread::autoFocusDone()
{
    LOG1("@%s", __FUNCTION__);
    Message msg;
    msg.id = MESSAGE_ID_AUTO_FOCUS_DONE;
    mMessageQueue.send(&msg);
}

status_t ControlThread::handleMessageExit()
{
    LOG1("@%s", __FUNCTION__);
    mThreadRunning = false;

    // TODO: any other cleanup that may need to be done

    return NO_ERROR;
}

status_t ControlThread::startPreviewCore(bool videoMode)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    int previewWidth;
    int previewHeight;
    int previewFormat;
    int videoWidth;
    int videoHeight;
    int videoFormat;
    State state;
    CameraDriver::Mode mode;

    if (mState != STATE_STOPPED) {
        ALOGE("Must be in STATE_STOPPED to start preview");
        return INVALID_OPERATION;
    }

    if (videoMode) {
        LOG1("Starting preview in video mode");
        state = STATE_PREVIEW_VIDEO;
        mode = CameraDriver::MODE_VIDEO;
    } else {
        LOG1("Starting preview in still mode");
        state = STATE_PREVIEW_STILL;
        mode = CameraDriver::MODE_PREVIEW;
    }

    previewFormat = V4L2Format(mParameters.getPreviewFormat());
    videoFormat = V4L2Format(mParameters.get(CameraParameters::KEY_VIDEO_FRAME_FORMAT));

    if (previewFormat != videoFormat) {
        ALOGE("preview and video format must be the same");
        return BAD_VALUE;
    }

    mParameters.getPreviewSize(&previewWidth, &previewHeight);
    mDriver->setPreviewFrameSize(previewWidth, previewHeight);
    mPreviewThread->setPreviewConfig(previewWidth, previewHeight, mCameraFormat, previewFormat);

    // set video frame config
    if (videoMode) {
        mParameters.getVideoSize(&videoWidth, &videoHeight);
        mDriver->setVideoFrameSize(videoWidth, videoHeight);
        mVideoThread->setConfig(mCameraFormat, videoFormat, videoWidth, videoHeight);
    }

    mPipeThread->setConfig(mCameraFormat, previewFormat, previewWidth, previewHeight);

    mNumBuffers = mDriver->getNumBuffers();
    mConversionBuffers = new CameraBuffer[mNumBuffers];
    int bytes = frameSize(previewFormat, previewWidth, previewHeight);
    for (int i = 0; i < mNumBuffers; i++) {
        mCallbacks->allocateMemory(&mConversionBuffers[i], bytes);
        mConversionBuffers[i].mID = i;
        mConversionBuffers[i].setFormat(previewFormat);
        mConversionBuffers[i].mType = BUFFER_TYPE_INTERMEDIATE;
        mConversionBuffers[i].mOwner = this;
        mFreeBuffers.push(&mConversionBuffers[i]);
    }

    // start the data flow
    status = mDriver->start(mode);
    if (status == NO_ERROR) {
        mState = state;
    } else {
        ALOGE("Error starting driver!");
    }

    return status;
}

status_t ControlThread::stopPreviewCore()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;

    status = mPipeThread->flushBuffers();
    if (status != NO_ERROR)
        ALOGE("error flushing pipe buffers");

    status = mPreviewThread->flushBuffers();
    if (status != NO_ERROR)
        ALOGE("error flushing preview buffers");

    status = mDriver->stop();
    if (status == NO_ERROR) {
        mState = STATE_STOPPED;
    } else {
        ALOGE("Error stopping driver in preview mode!");
    }

    for (int i = 0; i < mNumBuffers; i++) {
        mConversionBuffers[i].releaseMemory();
    }
    delete [] mConversionBuffers;
    mFreeBuffers.clear();
    mConversionBuffers = 0;
    mLastRecordingBuff = 0;

    return status;
}

status_t ControlThread::stopCapture()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;

    if (mState != STATE_CAPTURE) {
        ALOGE("Must be in STATE_CAPTURE to stop capture");
        return INVALID_OPERATION;
    }

    status = mPictureThread->flushBuffers();
    if (status != NO_ERROR) {
        ALOGE("Error flushing PictureThread!");
        return status;
    }

    status = mDriver->stop();
    if (status != NO_ERROR) {
        ALOGE("Error stopping driver!");
        return status;
    }

    mState = STATE_STOPPED;
    return status;
}

status_t ControlThread::restartPreview(bool videoMode)
{
    LOG1("@%s: mode = %s", __FUNCTION__, videoMode?"VIDEO":"STILL");
    bool faceActive = mFaceDetectionActive;
    stopFaceDetection(true);
    status_t status = stopPreviewCore();
    if (status == NO_ERROR)
        status = startPreviewCore(videoMode);
    if (faceActive)
        startFaceDetection();
    return status;
}

status_t ControlThread::handleMessageStartPreview()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    bool videoMode = isParameterSet(CameraParameters::KEY_RECORDING_HINT) ? true : false;
    if (mState == STATE_CAPTURE) {
        /*
        status = stopCapture();
        if (status != NO_ERROR) {
            ALOGE("Could not stop capture before start preview!");
            return status;
        }
        */
        if (videoMode) {
            mState = STATE_PREVIEW_VIDEO;
        } else {
            mState = STATE_PREVIEW_STILL;
        }
        mMessageQueue.reply(MESSAGE_ID_START_PREVIEW, status);
        return status;
    }
    if (mState == STATE_STOPPED) {
        // API says apps should call startFaceDetection when resuming preview
        // stop FD here to avoid accidental FD.
        stopFaceDetection();
        status = startPreviewCore(videoMode);
    } else {
        ALOGE("Error starting preview. Invalid state!");
        status = INVALID_OPERATION;
    }

    // return status and unblock message sender
    mMessageQueue.reply(MESSAGE_ID_START_PREVIEW, status);
    return status;
}

status_t ControlThread::handleMessageStopPreview()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    // In STATE_CAPTURE, preview is already stopped, nothing to do
    if (mState != STATE_CAPTURE) {
        stopFaceDetection(true);
        if (mState != STATE_STOPPED) {
            status = stopPreviewCore();
        } else {
            ALOGE("Error stopping preview. Invalid state!");
            status = INVALID_OPERATION;
        }
    }
    // return status and unblock message sender
    mMessageQueue.reply(MESSAGE_ID_STOP_PREVIEW, status);
    return status;
}

status_t ControlThread::handleMessageStartRecording()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;

    if (mState == STATE_PREVIEW_VIDEO) {
        mState = STATE_RECORDING;
    } else if (mState == STATE_PREVIEW_STILL) {
        /* We are in PREVIEW_STILL mode; in order to start recording
         * we first need to stop CameraDriver and restart it with MODE_VIDEO
         */
        LOG2("We are in STATE_PREVIEW. Switching to STATE_VIDEO before starting to record.");
        if ((status = stopPreviewCore()) == NO_ERROR) {
            if ((status = startPreviewCore(true)) == NO_ERROR) {
                mState = STATE_RECORDING;
            } else {
                ALOGE("Error starting driver in VIDEO mode!");
            }
        } else {
            ALOGE("Error stopping driver!");
        }
    } else {
        ALOGE("Error starting recording. Invalid state!");
        status = INVALID_OPERATION;
    }

    // return status and unblock message sender
    mMessageQueue.reply(MESSAGE_ID_START_RECORDING, status);
    return status;
}

status_t ControlThread::handleMessageStopRecording()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;

    if (mState == STATE_RECORDING) {
        mState = STATE_PREVIEW_VIDEO;
    } else {
        ALOGE("Error stopping recording. Invalid state!");
        status = INVALID_OPERATION;
    }

    // return status and unblock message sender
    mMessageQueue.reply(MESSAGE_ID_STOP_RECORDING, status);
    return status;
}

status_t ControlThread::handleMessageTakePicture()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    CameraBuffer *snapshotBuffer = 0, *postviewBuffer = 0;
    State origState = mState;
    int width, previewWidth;
    int height, previewHeight;

    if (origState != STATE_PREVIEW_STILL && origState != STATE_RECORDING) {
        ALOGE("we only support snapshot in still preview and recording");
        return INVALID_OPERATION;
    }

    //stopFaceDetection();

    if (origState == STATE_PREVIEW_STILL) {
        //status = stopPreviewCore();
        if (status != NO_ERROR) {
            ALOGE("Error stopping preview!");
            return status;
        }
        mState = STATE_CAPTURE;
    }

    // Get the current params
    mParameters.getPreviewSize(&previewWidth, &previewHeight);
    mParameters.getPictureSize(&width, &height);
    if (origState == STATE_RECORDING) {
        // override picture size to video size if recording
        int vidWidth, vidHeight;
        mDriver->getVideoSize(&vidWidth, &vidHeight);
        if (width != vidWidth || height != vidHeight) {
            ALOGW("Warning overriding snapshot size=%d,%d to %d,%d",
                    width, height, vidWidth, vidHeight);
            width = vidWidth;
            height = vidHeight;
        }
    }

    // see if we support thumbnail
    mThumbSupported = isThumbSupported(origState);

    // Configure PictureThread
    PictureThread::Config config;

    if (origState == STATE_PREVIEW_STILL) {
        gatherExifInfo(&mParameters, false, &config.exif);
    } else if (origState == STATE_RECORDING) { // STATE_RECORDING
        // Picture thread uses snapshot-size to configure itself. However,
        // if in recording mode we need to override snapshot with video-size.
        CameraParameters copyParams = mParameters;
        copyParams.setPictureSize(width, height); // make sure picture size is same as video size
        gatherExifInfo(&copyParams, false, &config.exif);
    }

    config.picture.format = mCameraFormat;
    config.picture.quality = mParameters.getInt(CameraParameters::KEY_JPEG_QUALITY);
    config.picture.width = width;
    config.picture.height = height;

    if (mThumbSupported) {
        config.thumbnail.format = mCameraFormat;
        config.thumbnail.quality = mParameters.getInt(CameraParameters::KEY_JPEG_THUMBNAIL_QUALITY);
        config.thumbnail.width = mParameters.getInt(CameraParameters::KEY_JPEG_THUMBNAIL_WIDTH);
        config.thumbnail.height = mParameters.getInt(CameraParameters::KEY_JPEG_THUMBNAIL_HEIGHT);
    }

    mPictureThread->setConfig(&config);

    if (origState == STATE_PREVIEW_STILL) {
        /*
        // Configure and start the driver
        mDriver->setSnapshotFrameSize(width, height);

        if ((status = mDriver->start(CameraDriver::MODE_CAPTURE)) != NO_ERROR) {
            ALOGE("Error starting the driver in CAPTURE mode!");
            return status;
        }
        */

        // Get the snapshot
        if ((status = mDriver->getSnapshot(&snapshotBuffer)) != NO_ERROR) {
            ALOGE("Error in grabbing snapshot!");
            return status;
        }
        snapshotBuffer->setOwner(this);
        snapshotBuffer->mType = BUFFER_TYPE_SNAPSHOT;

        /** Hack - since we no longer close/start the driver it will still provide frames
            at preview resolution. If the actual pic size is smaller resize the buffer
            **/
        if (width < previewWidth && height < previewHeight) {

            int w_multil = previewWidth / width;
            int h_multil = previewHeight / height;

            // resize YUYV
            int* data = (int*)snapshotBuffer->getData();
            for (int i = 0; i < height; i++) {
                for (int j = 0; j+3 < (width << 1); j+=4) {
                  data[(((width*i)<<1)+j)/4] = data[(((previewWidth*h_multil*i)<<1)+j*w_multil)/4];
                }
            }
        }

        if (mThumbSupported) {
            if ((status = mDriver->getThumbnail(snapshotBuffer, &postviewBuffer, width, height,
                config.thumbnail.width, config.thumbnail.height)) != NO_ERROR) {
                ALOGE("Error in grabbing thumbnail!");
            }
            postviewBuffer->setOwner(this);
            postviewBuffer->mType = BUFFER_TYPE_THUMBNAIL;
        }

        mCallbacks->shutterSound();

        if (mThumbSupported && postviewBuffer != NULL) {
            status = mPictureThread->encode(snapshotBuffer, postviewBuffer);
        } else {
            status = mPictureThread->encode(snapshotBuffer);
        }
    } else {
        // If we are in video mode we simply use the recording buffer for picture encoding
        // No need to stop, reconfigure, and restart the driver
        if (mLastRecordingBuff !=0)
            status = mPictureThread->encode(mLastRecordingBuff);
    }

    return status;
}

status_t ControlThread::handleMessageCancelPicture()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;

    // TODO: implement

    return status;
}

status_t ControlThread::handleMessageAutoFocus()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;

    //If the apps call autoFocus(AutoFocusCallback), the camera will stop sending face callbacks.
    // The last face callback indicates the areas used to do autofocus. After focus completes,
    // face detection will resume sending face callbacks.
    //If the apps call cancelAutoFocus(), the face callbacks will also resume.
    LOG2("auto focus is on");
    if (mFaceDetectionActive)
        disableMsgType(CAMERA_MSG_PREVIEW_METADATA);

    status = mDriver->autoFocus();

    autoFocusDone();

    return status;
}

status_t ControlThread::handleMessageCancelAutoFocus()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    status = mDriver->cancelAutoFocus();
    LOG2("auto focus is off");
    if (mFaceDetectionActive)
        enableMsgType(CAMERA_MSG_PREVIEW_METADATA);

    /*
     * The normal autoFocus sequence is:
     * - camera client is calling autoFocus (we run the AF sequence and lock AF)
     * - camera client is calling:
     *     - takePicture: AF is locked, so the picture will have the focus established
     *       in previous step. In this case, we have to reset the auto-focus to enabled
     *       when the camera client will call startPreview.
     *     - cancelAutoFocus: AF is locked, camera client no longer wants this focus position
     *       so we should switch back to auto-focus
     */
    mDriver->cancelAutoFocus();

    return status;
}

status_t ControlThread::handleMessageReleaseRecordingFrame(MessageReleaseRecordingFrame *msg)
{
    LOG2("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    if (mState == STATE_RECORDING) {
        CameraBuffer *buff = mDriver->findBuffer(msg->buff);
        if (buff == 0) {
            buff = findRecordingBuffer(msg->buff);
            if (buff == 0) {
                ALOGE("Could not find recording buffer: %p", msg->buff);
                return DEAD_OBJECT;
            }
        }
        buff->decrementReader();
        LOG2("Recording buffer released from encoder, buff id= %d", buff->getID());
    }
    return status;
}

status_t ControlThread::handleMessageReturnBuffer(MessageReturnBuffer *msg)
{
    status_t status = NO_ERROR;
    CameraBuffer *buff = msg->buff;
    BufferType type = buff->mType;
    LOG2("return buffer id = %d, type=%d", buff->getID(), type);

    if ((type != BUFFER_TYPE_INTERMEDIATE)
            && !(mDriver->isBufferValid(buff)))
        return DEAD_OBJECT;
    switch (type) {
    case BUFFER_TYPE_PREVIEW:
        status = returnPreviewBuffer(buff);
        break;
    case BUFFER_TYPE_VIDEO:
        status = returnVideoBuffer(buff);
        break;
    case BUFFER_TYPE_SNAPSHOT:
        status = returnSnapshotBuffer(buff);
        break;
    case BUFFER_TYPE_THUMBNAIL:
        status = returnThumbnailBuffer(buff);
        break;
    case BUFFER_TYPE_INTERMEDIATE:
        status = returnConversionBuffer(buff);
        break;
    default:
        ALOGE("invalid buffer type for buff %d", buff->getID());
        return UNKNOWN_ERROR;
    }

    return status;
}

status_t ControlThread::handleMessageAutoFocusDone()
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    if (mFaceDetectionActive)
        enableMsgType(CAMERA_MSG_PREVIEW_METADATA);
    // Implement post auto-focus functions

    enableMsgType(CAMERA_MSG_FOCUS);
    mCallbacks->autofocusDone(true);

    return status;
}

status_t ControlThread::validateParameters(const CameraParameters *params)
{
    LOG1("@%s: params = %p", __FUNCTION__, params);
    // PREVIEW
    int previewWidth, previewHeight;
    params->getPreviewSize(&previewWidth, &previewHeight);
    if (previewWidth <= 0 || previewHeight <= 0) {
        ALOGE("bad preview size");
        return BAD_VALUE;
    }

    int minFPS, maxFPS;
    params->getPreviewFpsRange(&minFPS, &maxFPS);
    if (minFPS > maxFPS || minFPS <= 0 || maxFPS <= 0) {
        ALOGE("invalid fps range [%d,%d]", minFPS, maxFPS);
        return BAD_VALUE;
    }

    // VIDEO
    int videoWidth, videoHeight;
    params->getPreviewSize(&videoWidth, &videoHeight);
    if (videoWidth <= 0 || videoHeight <= 0) {
        ALOGE("bad video size");
        return BAD_VALUE;
    }

    // SNAPSHOT
    int pictureWidth, pictureHeight;
    params->getPreviewSize(&pictureWidth, &pictureHeight);
    if (pictureWidth <= 0 || pictureHeight <= 0) {
        ALOGE("bad picture size");
        return BAD_VALUE;
    }

    // ZOOM
    int zoom = params->getInt(CameraParameters::KEY_ZOOM);
    int maxZoom = params->getInt(CameraParameters::KEY_MAX_ZOOM);
    if (zoom > maxZoom) {
        ALOGE("bad zoom index");
        return BAD_VALUE;
    }

    // FLASH
    const char* flashMode = params->get(CameraParameters::KEY_FLASH_MODE);
    const char* flashModes = params->get(CameraParameters::KEY_SUPPORTED_FLASH_MODES);
    if (strstr(flashModes, flashMode) == NULL) {
        ALOGE("bad flash mode");
        return BAD_VALUE;
    }

    // FOCUS
    const char* focusMode = params->get(CameraParameters::KEY_FOCUS_MODE);
    const char* focusModes = params->get(CameraParameters::KEY_SUPPORTED_FOCUS_MODES);
    if (strstr(focusModes, focusMode) == NULL) {
        ALOGE("bad focus mode");
        return BAD_VALUE;
    }

    // FOCUS WINDOWS
    int maxWindows = params->getInt(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS);
    if (maxWindows > 0) {
        const char *pFocusWindows = params->get(CameraParameters::KEY_FOCUS_AREAS);
        if (pFocusWindows && (strlen(pFocusWindows) > 0)) {
            LOG1("Scanning AF windows from params: %s", pFocusWindows);
            const char *argTail = pFocusWindows;
            int winCount = 0;
            CameraWindow focusWindow;
            while (argTail && winCount < maxWindows) {
                // String format: "(topleftx,toplefty,bottomrightx,bottomrighty,weight),(...)"
                int i = sscanf(argTail, "(%d,%d,%d,%d,%d)",
                        &focusWindow.x_left,
                        &focusWindow.y_top,
                        &focusWindow.x_right,
                        &focusWindow.y_bottom,
                        &focusWindow.weight);
                argTail = strchr(argTail + 1, '(');
                // Camera app sets invalid window 0,0,0,0,0 - let it slide
                if ( !focusWindow.x_left && !focusWindow.y_top &&
                    !focusWindow.x_right && !focusWindow.y_bottom &&
                    !focusWindow.weight) {
                  continue;
                }
                if (i != 5) {
                    ALOGE("bad focus window format");
                    return BAD_VALUE;
                }
                bool verified = verifyCameraWindow(focusWindow);
                if (!verified) {
                    ALOGE("bad focus window");
                    return BAD_VALUE;
                }
            }
            // make sure not too many windows defined (to pass CTS)
            if (argTail) {
                ALOGE("bad - too many focus windows or bad format for focus window string");
                return BAD_VALUE;
            }
        }
    }

    // METERING AREAS
    maxWindows = params->getInt(CameraParameters::KEY_MAX_NUM_METERING_AREAS);
    if (maxWindows > 0) {
        const char *pMeteringWindows = params->get(CameraParameters::KEY_METERING_AREAS);
        if (pMeteringWindows && (strlen(pMeteringWindows) > 0)) {
            LOG1("Scanning Metering windows from params: %s", pMeteringWindows);
            const char *argTail = pMeteringWindows;
            int winCount = 0;
            CameraWindow meteringWindow;
            while (argTail && winCount < maxWindows) {
                // String format: "(topleftx,toplefty,bottomrightx,bottomrighty,weight),(...)"
                int i = sscanf(argTail, "(%d,%d,%d,%d,%d)",
                        &meteringWindow.x_left,
                        &meteringWindow.y_top,
                        &meteringWindow.x_right,
                        &meteringWindow.y_bottom,
                        &meteringWindow.weight);
                argTail = strchr(argTail + 1, '(');
                // Camera app sets invalid window 0,0,0,0,0 - let it slide
                if ( !meteringWindow.x_left && !meteringWindow.y_top &&
                    !meteringWindow.x_right && !meteringWindow.y_bottom &&
                    !meteringWindow.weight) {
                  continue;
                }
                if (i != 5) {
                    ALOGE("bad metering window format");
                    return BAD_VALUE;
                }
                bool verified = verifyCameraWindow(meteringWindow);
                if (!verified) {
                    ALOGE("bad metering window");
                    return BAD_VALUE;
                }
            }
            // make sure not too many windows defined (to pass CTS)
            if (argTail) {
                ALOGE("bad - too many metering windows or bad format for metering window string");
                return BAD_VALUE;
            }
        }
    }

    // MISCELLANEOUS
    // TODO: implement validation for other features not listed above

    return NO_ERROR;
}

status_t ControlThread::processDynamicParameters(const CameraParameters *oldParams,
        CameraParameters *newParams)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    int oldZoom = oldParams->getInt(CameraParameters::KEY_ZOOM);
    int newZoom = newParams->getInt(CameraParameters::KEY_ZOOM);
    bool videoMode = isParameterSet(CameraParameters::KEY_RECORDING_HINT) ? true : false;

    if (oldZoom != newZoom)
        status = mDriver->setZoom(newZoom);

    // We won't take care of the status returned by the following calls since
    // failure of setting one parameter should not stop us setting the other parameters

    // Colour effect
    status = processParamEffect(oldParams, newParams);

    if (status == NO_ERROR) {
        // Scene Mode
        status = processParamFlash(oldParams, newParams);
    }

    if (status == NO_ERROR) {
        // Scene Mode
        status = processParamSceneMode(oldParams, newParams);
    }

    if (status == NO_ERROR) {
        //Focus Mode
        status = processParamFocusMode(oldParams, newParams);
    }

    if (status == NO_ERROR || !mFaceDetectionActive) {
        // white balance
        status = processParamWhiteBalance(oldParams, newParams);
    }

    if (status == NO_ERROR) {
        // ae lock
        status = processParamAELock(oldParams, newParams);
    }

    if (status == NO_ERROR) {
        // awb lock
        status = processParamAWBLock(oldParams, newParams);
    }

    if (!mFaceDetectionActive && status == NO_ERROR) {
        // customize metering
        status = processParamSetMeteringAreas(oldParams, newParams);
    }

    return status;
}

status_t ControlThread::processParamAWBLock(const CameraParameters *oldParams,
        CameraParameters *newParams)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;

    // awb lock mode
    const char* oldValue = oldParams->get(CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK);
    const char* newValue = newParams->get(CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK);
    if (newValue && oldValue && strncmp(newValue, oldValue, MAX_PARAM_VALUE_LENGTH) != 0) {
        bool awb_lock;

        if(!strncmp(newValue, CameraParameters::TRUE, strlen(CameraParameters::TRUE))) {
            awb_lock = true;
        } else if(!strncmp(newValue, CameraParameters::FALSE, strlen(CameraParameters::FALSE))) {
            awb_lock = false;
        } else {
            ALOGE("Invalid value received for %s: %s", CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK, newValue);
            return INVALID_OPERATION;
        }

        mDriver->setAwbLock(awb_lock);
        if (status == NO_ERROR) {
            LOG1("Changed: %s -> %s", CameraParameters::KEY_AUTO_WHITEBALANCE_LOCK, newValue);
        }
    }

    return status;
}

status_t ControlThread::processParamAELock(const CameraParameters *oldParams,
        CameraParameters *newParams)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;

    // ae lock mode
    const char* oldValue = oldParams->get(CameraParameters::KEY_AUTO_EXPOSURE_LOCK);
    const char* newValue = newParams->get(CameraParameters::KEY_AUTO_EXPOSURE_LOCK);

    if (newValue && oldValue && strncmp(newValue, oldValue, MAX_PARAM_VALUE_LENGTH) != 0) {
        bool ae_lock;

        if(!strncmp(newValue, CameraParameters::TRUE, strlen(CameraParameters::TRUE))) {
            ae_lock = true;
        } else  if(!strncmp(newValue, CameraParameters::FALSE, strlen(CameraParameters::FALSE))) {
            ae_lock = false;
        } else {
            ALOGE("Invalid value received for %s: %s", CameraParameters::KEY_AUTO_EXPOSURE_LOCK, newValue);
            return INVALID_OPERATION;
        }

        mDriver->setAeLock(ae_lock);
        if (status == NO_ERROR) {
            LOG1("Changed: %s -> %s", CameraParameters::KEY_AUTO_EXPOSURE_LOCK, newValue);
        }
    }

    return status;
}

status_t ControlThread::processParamFlash(const CameraParameters *oldParams,
        CameraParameters *newParams)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    const char* oldValue = oldParams->get(CameraParameters::KEY_FLASH_MODE);
    const char* newValue = newParams->get(CameraParameters::KEY_FLASH_MODE);
    CameraDriver::FlashMode flashMode;
    if (newValue && oldValue && strncmp(newValue, oldValue, MAX_PARAM_VALUE_LENGTH) != 0) {
        if (!strncmp(newValue, CameraParameters::FLASH_MODE_AUTO, strlen(CameraParameters::FLASH_MODE_AUTO))) {
            flashMode = CameraDriver::FLASH_MODE_AUTO;
        } else if (!strncmp(newValue, CameraParameters::FLASH_MODE_OFF, strlen(CameraParameters::FLASH_MODE_OFF))) {
            flashMode = CameraDriver::FLASH_MODE_OFF;
        } else if (!strncmp(newValue, CameraParameters::FLASH_MODE_ON, strlen(CameraParameters::FLASH_MODE_ON))) {
            flashMode = CameraDriver::FLASH_MODE_ON;
        } else if (!strncmp(newValue, CameraParameters::FLASH_MODE_TORCH, strlen(CameraParameters::FLASH_MODE_TORCH))) {
            flashMode = CameraDriver::FLASH_MODE_TORCH;
        } else if (!strncmp(newValue, CameraParameters::FLASH_MODE_TORCH, strlen(CameraParameters::FLASH_MODE_RED_EYE))) {
            flashMode = CameraDriver::FLASH_MODE_RED_EYE;
        } else {
            ALOGE("Invalid flash mode");
            return BAD_VALUE;
        }

        mDriver->setFlashMode(flashMode);
        if (status == NO_ERROR) {
            LOG1("Changed: %s -> %s", CameraParameters::KEY_FLASH_MODE, newValue);
        }
    }
    return status;
}

status_t ControlThread::processParamEffect(const CameraParameters *oldParams,
        CameraParameters *newParams)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    const char* oldEffect = oldParams->get(CameraParameters::KEY_EFFECT);
    const char* newEffect = newParams->get(CameraParameters::KEY_EFFECT);

    if (newEffect && oldEffect && strncmp(newEffect, oldEffect, MAX_PARAM_VALUE_LENGTH) != 0) {

        CameraDriver::Effect effect;
        if (!strncmp(newEffect, CameraParameters::EFFECT_NONE, strlen(CameraParameters::EFFECT_NONE))) {
            effect = CameraDriver::EFFECT_NONE;
        } else if (!strncmp(newEffect, CameraParameters::EFFECT_MONO, strlen(CameraParameters::EFFECT_MONO))) {
            effect = CameraDriver::EFFECT_MONO;
        } else if (!strncmp(newEffect, CameraParameters::EFFECT_NEGATIVE, strlen(CameraParameters::EFFECT_NEGATIVE))) {
            effect = CameraDriver::EFFECT_NEGATIVE;
        } else if (!strncmp(newEffect, CameraParameters::EFFECT_SOLARIZE, strlen(CameraParameters::EFFECT_SOLARIZE))) {
            effect = CameraDriver::EFFECT_SOLARIZE;
        } else if (!strncmp(newEffect, CameraParameters::EFFECT_SEPIA, strlen(CameraParameters::EFFECT_SEPIA))) {
            effect = CameraDriver::EFFECT_SEPIA;
        } else if (!strncmp(newEffect, CameraParameters::EFFECT_POSTERIZE, strlen(CameraParameters::EFFECT_POSTERIZE))) {
            effect = CameraDriver::EFFECT_POSTERIZE;
        } else if (!strncmp(newEffect, CameraParameters::EFFECT_WHITEBOARD, strlen(CameraParameters::EFFECT_WHITEBOARD))) {
            effect = CameraDriver::EFFECT_WHITEBOARD;
        } else if (!strncmp(newEffect, CameraParameters::EFFECT_BLACKBOARD, strlen(CameraParameters::EFFECT_BLACKBOARD))) {
            effect = CameraDriver::EFFECT_BLACKBOARD;
        } else if (!strncmp(newEffect, CameraParameters::EFFECT_AQUA, strlen(CameraParameters::EFFECT_AQUA))) {
            effect = CameraDriver::EFFECT_AQUA;
        } else {
            ALOGE("Invalid color effect");
            return BAD_VALUE;
        }

        status = mDriver->setEffect(effect);
        if (status == NO_ERROR) {
            LOG1("Changed: %s -> %s", CameraParameters::KEY_EFFECT, newEffect);
        }
    }
    return status;
}

status_t ControlThread::processParamSceneMode(const CameraParameters *oldParams,
        CameraParameters *newParams)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    const char* oldScene = oldParams->get(CameraParameters::KEY_SCENE_MODE);
    const char* newScene = newParams->get(CameraParameters::KEY_SCENE_MODE);
    if (newScene && oldScene && strncmp(newScene, oldScene, MAX_PARAM_VALUE_LENGTH) != 0) {
        CameraDriver::SceneMode scene;
        if (!strncmp (newScene, CameraParameters::SCENE_MODE_AUTO, strlen(CameraParameters::SCENE_MODE_AUTO))) {
            scene = CameraDriver::SCENE_MODE_AUTO;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_ACTION, strlen(CameraParameters::SCENE_MODE_ACTION))) {
            scene = CameraDriver::SCENE_MODE_ACTION;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_PORTRAIT, strlen(CameraParameters::SCENE_MODE_PORTRAIT))) {
            scene = CameraDriver::SCENE_MODE_PORTRAIT;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_LANDSCAPE, strlen(CameraParameters::SCENE_MODE_LANDSCAPE))) {
            scene = CameraDriver::SCENE_MODE_LANDSCAPE;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_NIGHT, strlen(CameraParameters::SCENE_MODE_NIGHT))) {
            scene = CameraDriver::SCENE_MODE_NIGHT;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_NIGHT_PORTRAIT, strlen(CameraParameters::SCENE_MODE_PORTRAIT))) {
            scene = CameraDriver::SCENE_MODE_PORTRAIT;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_THEATRE, strlen(CameraParameters::SCENE_MODE_THEATRE))) {
            scene = CameraDriver::SCENE_MODE_THEATRE;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_BEACH, strlen(CameraParameters::SCENE_MODE_BEACH))) {
            scene = CameraDriver::SCENE_MODE_BEACH;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_SNOW, strlen(CameraParameters::SCENE_MODE_SNOW))) {
            scene = CameraDriver::SCENE_MODE_SNOW;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_SUNSET, strlen(CameraParameters::SCENE_MODE_SUNSET))) {
            scene = CameraDriver::SCENE_MODE_SUNSET;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_STEADYPHOTO, strlen(CameraParameters::SCENE_MODE_STEADYPHOTO))) {
            scene = CameraDriver::SCENE_MODE_STEADYPHOTO;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_FIREWORKS, strlen(CameraParameters::SCENE_MODE_FIREWORKS))) {
            scene = CameraDriver::SCENE_MODE_FIREWORKS;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_SPORTS, strlen(CameraParameters::SCENE_MODE_SPORTS))) {
            scene = CameraDriver::SCENE_MODE_SPORTS;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_PARTY, strlen(CameraParameters::SCENE_MODE_PARTY))) {
            scene = CameraDriver::SCENE_MODE_PARTY;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_CANDLELIGHT, strlen(CameraParameters::SCENE_MODE_CANDLELIGHT))) {
            scene = CameraDriver::SCENE_MODE_CANDLELIGHT;
        } else if (!strncmp (newScene, CameraParameters::SCENE_MODE_BARCODE, strlen(CameraParameters::SCENE_MODE_BARCODE))) {
            scene = CameraDriver::SCENE_MODE_BARCODE;
        } else {
            ALOGE("Invalid scene mode");
            return BAD_VALUE;
        }

        mDriver->setSceneMode(scene);
        if (status == NO_ERROR) {
            LOG1("Changed: %s -> %s", CameraParameters::KEY_SCENE_MODE, newScene);
        }
    }
    return status;
}

bool ControlThread::verifyCameraWindow(const CameraWindow &win)
{
    if (win.x_right <= win.x_left ||
        win.y_bottom <= win.y_top)
        return false;
    if ( (win.y_top < -1000) || (win.y_top > 1000) ) return false;
    if ( (win.y_bottom < -1000) || (win.y_bottom > 1000) ) return false;
    if ( (win.x_right < -1000) || (win.x_right > 1000) ) return false;
    if ( (win.x_left < -1000) || (win.x_left > 1000) ) return false;
    if ( (win.weight < 1) || (win.weight > 1000) ) return false;
    return true;
}

void ControlThread::preSetCameraWindows(CameraWindow* focusWindows, size_t winCount)
{
    LOG1("@%s", __FUNCTION__);
    // Camera KEY_FOCUS_AREAS Coordinates range from -1000 to 1000.
    if (winCount > 0) {
        int width;
        int height;
        const int FOCUS_AREAS_X_OFFSET = 1000;
        const int FOCUS_AREAS_Y_OFFSET = 1000;
        const int FOCUS_AREAS_WIDTH = 2000;
        const int FOCUS_AREAS_HEIGHT = 2000;
        const int WINDOWS_TOTAL_WEIGHT = 16;
        mParameters.getPreviewSize(&width, &height);
        size_t windowsWeight = 0;
        for(size_t i = 0; i < winCount; i++){
            windowsWeight += focusWindows[i].weight;
        }
        if(!windowsWeight)
            windowsWeight = 1;

        size_t weight_sum = 0;
        for(size_t i =0; i < winCount; i++) {
            // skip all zero value
            focusWindows[i].x_left = (focusWindows[i].x_left + FOCUS_AREAS_X_OFFSET) * (width - 1) / FOCUS_AREAS_WIDTH;
            focusWindows[i].x_right = (focusWindows[i].x_right + FOCUS_AREAS_X_OFFSET) * (width - 1) / FOCUS_AREAS_WIDTH;
            focusWindows[i].y_top = (focusWindows[i].y_top + FOCUS_AREAS_Y_OFFSET) * (height - 1) / FOCUS_AREAS_HEIGHT;
            focusWindows[i].y_bottom = (focusWindows[i].y_bottom + FOCUS_AREAS_Y_OFFSET) * (height - 1) / FOCUS_AREAS_HEIGHT;
            focusWindows[i].weight = focusWindows[i].weight * WINDOWS_TOTAL_WEIGHT / windowsWeight;
            weight_sum += focusWindows[i].weight;
            LOG1("Preset camera window %d: (%d,%d,%d,%d,%d)",
                    i,
                    focusWindows[i].x_left,
                    focusWindows[i].y_top,
                    focusWindows[i].x_right,
                    focusWindows[i].y_bottom,
                    focusWindows[i].weight);
        }
        //weight sum value should be WINDOWS_TOTAL_WEIGHT
        focusWindows[winCount-1].weight += WINDOWS_TOTAL_WEIGHT - weight_sum;
    }
}

status_t ControlThread::processParamFocusMode(const CameraParameters *oldParams,
        CameraParameters *newParams)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    const char* oldFocus = oldParams->get(CameraParameters::KEY_FOCUS_MODE);
    const char* newFocus = newParams->get(CameraParameters::KEY_FOCUS_MODE);
    CameraDriver::FocusMode focus;
    if (newFocus) {
        if (!strncmp(newFocus, CameraParameters::FOCUS_MODE_INFINITY, strlen(CameraParameters::FOCUS_MODE_INFINITY))) {
            focus = CameraDriver::FOCUS_MODE_INFINITY;
        } else if (!strncmp(newFocus, CameraParameters::FOCUS_MODE_MACRO, strlen(CameraParameters::FOCUS_MODE_MACRO))) {
            focus = CameraDriver::FOCUS_MODE_MACRO;
        } else if (!strncmp(newFocus, CameraParameters::FOCUS_MODE_AUTO, strlen(CameraParameters::FOCUS_MODE_AUTO))) {
            focus = CameraDriver::FOCUS_MODE_AUTO;
        } else if (!strncmp(newFocus, CameraParameters::FOCUS_MODE_FIXED, strlen(CameraParameters::FOCUS_MODE_FIXED))) {
            focus = CameraDriver::FOCUS_MODE_FIXED;
        } else if (!strncmp(newFocus, CameraParameters::FOCUS_MODE_EDOF, strlen(CameraParameters::FOCUS_MODE_EDOF))) {
            focus = CameraDriver::FOCUS_MODE_EDOF;
        } else if (!strncmp(newFocus, CameraParameters::FOCUS_MODE_CONTINUOUS_VIDEO, strlen(CameraParameters::FOCUS_MODE_CONTINUOUS_VIDEO))) {
            focus = CameraDriver::FOCUS_MODE_CONTINUOUS_VIDEO;
        } else if (!strncmp(newFocus, CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE, strlen(CameraParameters::FOCUS_MODE_CONTINUOUS_PICTURE))) {
            focus = CameraDriver::FOCUS_MODE_CONTINUOUS_PICTURE;
        } else {
            ALOGE("Bad focus value");
            return BAD_VALUE;
        }
    } else {
        ALOGE("NULL focus value");
        return BAD_VALUE;
    }

    // Handling the window information in auto, macro and continuous video mode.
    // If focus window is set, we will actually use the touch mode!
    CameraWindow *focusWindows = NULL;
    size_t winCount = 0;
    if (focus == CameraDriver::FOCUS_MODE_AUTO ||
            focus == CameraDriver::FOCUS_MODE_CONTINUOUS_VIDEO ||
            focus == CameraDriver::FOCUS_MODE_MACRO) {

        // See if any focus windows are set
        const char *pFocusWindows = newParams->get(CameraParameters::KEY_FOCUS_AREAS);
        size_t maxWindows = newParams->getInt(CameraParameters::KEY_MAX_NUM_FOCUS_AREAS);

        if (!mFaceDetectionActive && pFocusWindows && (maxWindows > 0) && (strlen(pFocusWindows) > 0)) {
            LOG1("Scanning AF windows from params: %s", pFocusWindows);
            const char *argTail = pFocusWindows;
            focusWindows = new CameraWindow[maxWindows];
            while (argTail && winCount < maxWindows) {
                // String format: "(topleftx,toplefty,bottomrightx,bottomrighty,weight),(...)"
                int i = sscanf(argTail, "(%d,%d,%d,%d,%d)",
                        &focusWindows[winCount].x_left,
                        &focusWindows[winCount].y_top,
                        &focusWindows[winCount].x_right,
                        &focusWindows[winCount].y_bottom,
                        &focusWindows[winCount].weight);
                if (i != 5)
                    break;
                bool verified = verifyCameraWindow(focusWindows[winCount]);
                LOG1("\tWindow %d (%d,%d,%d,%d,%d) [%s]",
                        winCount,
                        focusWindows[winCount].x_left,
                        focusWindows[winCount].y_top,
                        focusWindows[winCount].x_right,
                        focusWindows[winCount].y_bottom,
                        focusWindows[winCount].weight,
                        (verified)?"GOOD":"IGNORED");
                argTail = strchr(argTail + 1, '(');
                if (verified) {
                    winCount++;
                } else {
                    ALOGW("Ignoring invalid focus area: (%d,%d,%d,%d,%d)",
                            focusWindows[winCount].x_left,
                            focusWindows[winCount].y_top,
                            focusWindows[winCount].x_right,
                            focusWindows[winCount].y_bottom,
                            focusWindows[winCount].weight);
                }
            }
        }
    }

    if (winCount) {
        preSetCameraWindows(focusWindows, winCount);
        status = mDriver->setFocusMode(focus, focusWindows, winCount);
        delete focusWindows;
    } else {
        status = mDriver->setFocusMode(focus);
    }

    return status;
}

status_t ControlThread:: processParamSetMeteringAreas(const CameraParameters *oldParams,
        CameraParameters *newParams)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    size_t maxWindows = newParams->getInt(CameraParameters::KEY_MAX_NUM_METERING_AREAS);
    CameraWindow *meteringWindows = NULL;
    const char *pMeteringWindows = newParams->get(CameraParameters::KEY_METERING_AREAS);
    if (pMeteringWindows && (maxWindows > 0) && (strlen(pMeteringWindows) > 0)) {
        LOG1("Scanning AE metering from params: %s", pMeteringWindows);
        const char *argTail = pMeteringWindows;
        size_t winCount = 0;
        meteringWindows = new CameraWindow[maxWindows];
        while (argTail && winCount < maxWindows) {
            // String format: "(topleftx,toplefty,bottomrightx,bottomrighty,weight),(...)"
            int len = sscanf(argTail, "(%d,%d,%d,%d,%d)",
                    &meteringWindows[winCount].x_left,
                    &meteringWindows[winCount].y_top,
                    &meteringWindows[winCount].x_right,
                    &meteringWindows[winCount].y_bottom,
                    &meteringWindows[winCount].weight);
            if (len != 5)
                break;
            bool verified = verifyCameraWindow(meteringWindows[winCount]);
            LOG1("\tWindow %d (%d,%d,%d,%d,%d) [%s]",
                    winCount,
                    meteringWindows[winCount].x_left,
                    meteringWindows[winCount].y_top,
                    meteringWindows[winCount].x_right,
                    meteringWindows[winCount].y_bottom,
                    meteringWindows[winCount].weight,
                    (verified)?"GOOD":"IGNORED");
            argTail = strchr(argTail + 1, '(');
            if (verified) {
                winCount++;
            } else {
                ALOGW("Ignoring invalid metering area: (%d,%d,%d,%d,%d)",
                        meteringWindows[winCount].x_left,
                        meteringWindows[winCount].y_top,
                        meteringWindows[winCount].x_right,
                        meteringWindows[winCount].y_bottom,
                        meteringWindows[winCount].weight);
            }
        }
        // Looks like metering window(s) were set
        if (winCount > 0) {
            preSetCameraWindows(meteringWindows, winCount);
            status = mDriver->setMeteringAreas(meteringWindows, winCount);
            delete meteringWindows;
        }
    }
    return status;
}

status_t ControlThread::processParamWhiteBalance(const CameraParameters *oldParams,
        CameraParameters *newParams)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    const char* oldWb = oldParams->get(CameraParameters::KEY_WHITE_BALANCE);
    const char* newWb = newParams->get(CameraParameters::KEY_WHITE_BALANCE);
    if (newWb && oldWb && strncmp(newWb, oldWb, MAX_PARAM_VALUE_LENGTH) != 0) {

        CameraDriver::WhiteBalanceMode wbMode;
        if (!strncmp(newWb, CameraParameters::WHITE_BALANCE_AUTO, strlen(CameraParameters::WHITE_BALANCE_AUTO))) {
            wbMode = CameraDriver::WHITE_BALANCE_AUTO;
        } else if (!strncmp(newWb, CameraParameters::WHITE_BALANCE_INCANDESCENT, strlen(CameraParameters::WHITE_BALANCE_INCANDESCENT))) {
            wbMode = CameraDriver::WHITE_BALANCE_INCANDESCENT;
        } else if (!strncmp(newWb, CameraParameters::WHITE_BALANCE_WARM_FLUORESCENT, strlen(CameraParameters::WHITE_BALANCE_WARM_FLUORESCENT))) {
            wbMode = CameraDriver::WHITE_BALANCE_FLUORESCENT;
        } else if (!strncmp(newWb, CameraParameters::WHITE_BALANCE_DAYLIGHT, strlen(CameraParameters::WHITE_BALANCE_DAYLIGHT))) {
            wbMode = CameraDriver::WHITE_BALANCE_DAYLIGHT;
        } else if (!strncmp(newWb, CameraParameters::WHITE_BALANCE_CLOUDY_DAYLIGHT, strlen(CameraParameters::WHITE_BALANCE_CLOUDY_DAYLIGHT))) {
            wbMode = CameraDriver::WHITE_BALANCE_CLOUDY_DAYLIGHT;
        } else if (!strncmp(newWb, CameraParameters::WHITE_BALANCE_TWILIGHT, strlen(CameraParameters::WHITE_BALANCE_TWILIGHT))) {
            wbMode = CameraDriver::WHITE_BALANCE_TWILIGHT;
        } else if (!strncmp(newWb, CameraParameters::WHITE_BALANCE_SHADE, strlen(CameraParameters::WHITE_BALANCE_SHADE))) {
            wbMode = CameraDriver::WHITE_BALANCE_SHADE;
        } else if (!strncmp(newWb, CameraParameters::WHITE_BALANCE_FLUORESCENT, strlen(CameraParameters::WHITE_BALANCE_FLUORESCENT))) {
            wbMode = CameraDriver::WHITE_BALANCE_FLUORESCENT;
        } else {
            ALOGE("invalid wb mode");
            return BAD_VALUE;
        }

        mDriver->setWhiteBalanceMode(wbMode);
        if (status == NO_ERROR) {
            LOG1("Changed: %s -> %s", CameraParameters::KEY_WHITE_BALANCE, newWb);
        }
    }
    return status;
}

status_t ControlThread::processStaticParameters(const CameraParameters *oldParams,
        CameraParameters *newParams)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    bool previewFormatChanged = false;
    float previewAspectRatio = 0.0f;
    float videoAspectRatio = 0.0f;
    Vector<Size> sizes;
    bool videoMode = isParameterSet(CameraParameters::KEY_RECORDING_HINT) ? true : false;

    int oldWidth, newWidth;
    int oldHeight, newHeight;
    int previewWidth, previewHeight;
    int oldFormat, newFormat;

    // see if preview params have changed
    newParams->getPreviewSize(&newWidth, &newHeight);
    oldParams->getPreviewSize(&oldWidth, &oldHeight);
    newFormat = V4L2Format(newParams->getPreviewFormat());
    oldFormat = V4L2Format(oldParams->getPreviewFormat());
    previewWidth = oldWidth;
    previewHeight = oldHeight;
    if (newWidth != oldWidth || newHeight != oldHeight ||
            oldFormat != newFormat) {
        previewWidth = newWidth;
        previewHeight = newHeight;
        previewAspectRatio = 1.0 * newWidth / newHeight;
        LOG1("Preview size/format is changing: old=%dx%d %s; new=%dx%d %s; ratio=%.3f",
                oldWidth, oldHeight, v4l2Fmt2Str(oldFormat),
                newWidth, newHeight, v4l2Fmt2Str(newFormat),
                previewAspectRatio);
        previewFormatChanged = true;
    } else {
        previewAspectRatio = 1.0 * oldWidth / oldHeight;
        LOG1("Preview size/format is unchanged: old=%dx%d %s; ratio=%.3f",
                oldWidth, oldHeight, v4l2Fmt2Str(oldFormat),
                previewAspectRatio);
    }

    // see if video params have changed
    newParams->getVideoSize(&newWidth, &newHeight);
    oldParams->getVideoSize(&oldWidth, &oldHeight);
    if (newWidth != oldWidth || newHeight != oldHeight) {
        videoAspectRatio = 1.0 * newWidth / newHeight;
        LOG1("Video size is changing: old=%dx%d; new=%dx%d; ratio=%.3f",
                oldWidth, oldHeight,
                newWidth, newHeight,
                videoAspectRatio);
        previewFormatChanged = true;
        /*
         *  Camera client requested a new video size, so make sure that requested
         *  video size matches requested preview size. If it does not, then select
         *  a corresponding preview size to match the aspect ratio with video
         *  aspect ratio. Also, the video size must be at least as preview size
         */
        if (fabsf(videoAspectRatio - previewAspectRatio) > ASPECT_TOLERANCE) {
            ALOGW("Requested video (%dx%d) aspect ratio does not match preview \
                 (%dx%d) aspect ratio! The preview will be stretched!",
                    newWidth, newHeight,
                    previewWidth, previewHeight);
        }
    } else {
        videoAspectRatio = 1.0 * oldWidth / oldHeight;
        LOG1("Video size is unchanged: old=%dx%d; ratio=%.3f",
                oldWidth, oldHeight,
                videoAspectRatio);
        /*
         *  Camera client did not specify any video size, so make sure that
         *  requested preview size matches our default video size. If it does
         *  not, then select a corresponding video size to match the aspect
         *  ratio with preview aspect ratio.
         */
        if (fabsf(videoAspectRatio - previewAspectRatio) > ASPECT_TOLERANCE) {
            LOG1("Our video (%dx%d) aspect ratio does not match preview (%dx%d) aspect ratio!",
                    newWidth, newHeight,
                    previewWidth, previewHeight);
            newParams->getSupportedVideoSizes(sizes);
            for (size_t i = 0; i < sizes.size(); i++) {
                float thisSizeAspectRatio = 1.0 * sizes[i].width / sizes[i].height;
                if (fabsf(thisSizeAspectRatio - previewAspectRatio) <= ASPECT_TOLERANCE) {
                    if (sizes[i].width < previewWidth || sizes[i].height < previewHeight) {
                        // This video size is smaller than preview, can't use it
                        continue;
                    }
                    newWidth = sizes[i].width;
                    newHeight = sizes[i].height;
                    LOG1("Forcing video to %dx%d to match preview aspect ratio!", newWidth, newHeight);
                    newParams->setVideoSize(newWidth, newHeight);
                    break;
                }
            }
        }
    }

    // if preview is running and static params have changed, then we need
    // to stop, reconfigure, and restart the driver and all threads.
    if (previewFormatChanged) {
        switch (mState) {
        case STATE_PREVIEW_VIDEO:
        case STATE_PREVIEW_STILL:
            status = restartPreview(videoMode);
            break;
        case STATE_STOPPED:
            break;
        default:
            ALOGE("formats can only be changed while in preview or stop states");
            break;
        };
    }

    return status;
}

status_t ControlThread::handleMessageSetParameters(MessageSetParameters *msg)
{
    LOG1("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    CameraParameters newParams;
    CameraParameters oldParams = mParameters;
    String8 str_params(msg->params);
    newParams.unflatten(str_params);

    // Workaround: The camera firmware doesn't support preview dimensions that
    // are bigger than video dimensions. If a single preview dimension is larger
    // than the video dimension then the FW will downscale the preview resolution
    // to that of the video resolution.
    if (mState == STATE_PREVIEW_VIDEO || mState == STATE_RECORDING) {

        int pWidth, pHeight;
        int vWidth, vHeight;

        newParams.getPreviewSize(&pWidth, &pHeight);
        newParams.getVideoSize(&vWidth, &vHeight);
        if (vWidth < pWidth || vHeight < pHeight) {
            ALOGW("Warning: Video dimension(s) is smaller than preview dimension(s). "
                    "Overriding preview resolution to video resolution [%d, %d] --> [%d, %d]",
                    pWidth, pHeight, vWidth, vHeight);
            newParams.setPreviewSize(vWidth, vHeight);
        }
    }

    // print all old and new params for comparison (debug)
    LOG1("----------BEGIN OLD PARAMS----------");
    mParameters.dump();
    LOG1("---------- END OLD PARAMS ----------");
    LOG1("----------BEGIN NEW PARAMS----------");
    newParams.dump();
    LOG1("---------- END NEW PARAMS ----------");

    status = validateParameters(&newParams);
    if (status != NO_ERROR)
        goto exit;

    mParameters = newParams;

    // Take care of parameters that need to be set while the driver is stopped
    status = processStaticParameters(&oldParams, &newParams);
    if (status != NO_ERROR)
        goto exit;

    // Take care of parameters that can be set while driver is running
    status = processDynamicParameters(&oldParams, &newParams);
    if (status != NO_ERROR)
        goto exit;

    mParameters = newParams;

exit:
    // return status and unblock message sender
    mMessageQueue.reply(MESSAGE_ID_SET_PARAMETERS, status);
    return status;
}

status_t ControlThread::handleMessageGetParameters(MessageGetParameters *msg)
{
    status_t status = BAD_VALUE;

    if (msg->params) {
        // let app know if we support zoom in the preview mode indicated
        bool videoMode = isParameterSet(CameraParameters::KEY_RECORDING_HINT) ? true : false;
        CameraDriver::Mode mode = videoMode ? CameraDriver::MODE_VIDEO : CameraDriver::MODE_PREVIEW;
        mDriver->getZoomRatios(mode, &mParameters);
        mDriver->getFocusDistances(&mParameters);

        String8 params = mParameters.flatten();
        int len = params.length();
        *msg->params = strndup(params.string(), sizeof(char) * len);
        status = NO_ERROR;
    }
    mMessageQueue.reply(MESSAGE_ID_GET_PARAMETERS, status);
    return status;
}
status_t ControlThread::handleMessageCommand(MessageCommand* msg)
{
    status_t status = BAD_VALUE;
    switch (msg->cmd_id)
    {
    case CAMERA_CMD_START_FACE_DETECTION:
        status = startFaceDetection();
        break;
    case CAMERA_CMD_STOP_FACE_DETECTION:
        status = stopFaceDetection();
        break;
    default:
        break;
    }
    return status;
}
/**
 * From Android API:
 * Starts the face detection. This should be called after preview is started.
 * The camera will notify Camera.FaceDetectionListener
 *  of the detected faces in the preview frame. The detected faces may be the same as
 *  the previous ones.
 *
 *  Applications should call stopFaceDetection() to stop the face detection.
 *
 *  This method is supported if getMaxNumDetectedFaces() returns a number larger than 0.
 *  If the face detection has started, apps should not call this again.
 *  When the face detection is running, setWhiteBalance(String), setFocusAreas(List),
 *  and setMeteringAreas(List) have no effect.
 *  The camera uses the detected faces to do auto-white balance, auto exposure, and autofocus.
 *
 *  If the apps call autoFocus(AutoFocusCallback), the camera will stop sending face callbacks.
 *
 *  The last face callback indicates the areas used to do autofocus.
 *  After focus completes, face detection will resume sending face callbacks.
 *
 *  If the apps call cancelAutoFocus(), the face callbacks will also resume.
 *
 *  After calling takePicture(Camera.ShutterCallback, Camera.PictureCallback, Camera.PictureCallback)
 *  or stopPreview(), and then resuming preview with startPreview(),
 *  the apps should call this method again to resume face detection.
 *
 */
status_t ControlThread::startFaceDetection()
{
    LOG2("@%s", __FUNCTION__);
    if (mState == STATE_STOPPED || mFaceDetectionActive) {
        return INVALID_OPERATION;
    }
    if (m_pFaceDetector != 0) {
        m_pFaceDetector->start();
        mFaceDetectionActive = true;
        enableMsgType(CAMERA_MSG_PREVIEW_METADATA);
        return NO_ERROR;
    } else{
        return INVALID_OPERATION;
    }
}

status_t ControlThread::stopFaceDetection(bool wait)
{
    LOG2("@%s", __FUNCTION__);
    if( !mFaceDetectionActive )
        return NO_ERROR;
    mFaceDetectionActive = false;
    disableMsgType(CAMERA_MSG_PREVIEW_METADATA);
    if (m_pFaceDetector != 0) {
        m_pFaceDetector->stop(wait);
        return NO_ERROR;
    } else{
        return INVALID_OPERATION;
    }
}

status_t ControlThread::waitForAndExecuteMessage()
{
    LOG2("@%s", __FUNCTION__);
    status_t status = NO_ERROR;
    Message msg;
    mMessageQueue.receive(&msg);

    switch (msg.id) {

        case MESSAGE_ID_EXIT:
            status = handleMessageExit();
            break;

        case MESSAGE_ID_START_PREVIEW:
            status = handleMessageStartPreview();
            break;

        case MESSAGE_ID_STOP_PREVIEW:
            status = handleMessageStopPreview();
            break;

        case MESSAGE_ID_START_RECORDING:
            status = handleMessageStartRecording();
            break;

        case MESSAGE_ID_STOP_RECORDING:
            status = handleMessageStopRecording();
            break;

        case MESSAGE_ID_TAKE_PICTURE:
            status = handleMessageTakePicture();
            break;

        case MESSAGE_ID_CANCEL_PICTURE:
            status = handleMessageCancelPicture();
            break;

        case MESSAGE_ID_AUTO_FOCUS:
            status = handleMessageAutoFocus();
            break;

        case MESSAGE_ID_CANCEL_AUTO_FOCUS:
            status = handleMessageCancelAutoFocus();
            break;

        case MESSAGE_ID_RELEASE_RECORDING_FRAME:
            status = handleMessageReleaseRecordingFrame(&msg.data.releaseRecordingFrame);
            break;

        case MESSAGE_ID_RETURN_BUFFER:
            status = handleMessageReturnBuffer(&msg.data.returnBuffer);
            break;

        case MESSAGE_ID_AUTO_FOCUS_DONE:
            status = handleMessageAutoFocusDone();
            break;

        case MESSAGE_ID_SET_PARAMETERS:
            status = handleMessageSetParameters(&msg.data.setParameters);
            break;

        case MESSAGE_ID_GET_PARAMETERS:
            status = handleMessageGetParameters(&msg.data.getParameters);
            break;
        case MESSAGE_ID_COMMAND:
            status = handleMessageCommand(&msg.data.command);
            break;
        default:
            ALOGE("Invalid message");
            status = BAD_VALUE;
            break;
    };

    if (status != NO_ERROR)
        ALOGE("Error handling message: %d", (int) msg.id);
    return status;
}

CameraBuffer* ControlThread::findRecordingBuffer(void *findMe)
{
    // This is a small list, so incremental search is not an issue right now
    if (mConversionBuffers) {
        for (int i = 0; i < mNumBuffers; i++) {
            if (mConversionBuffers[i].getData() == findMe)
                return &mConversionBuffers[i];
        }
    }
    return 0;
}

status_t ControlThread::dequeuePreview()
{
    LOG2("@%s", __FUNCTION__);
    CameraBuffer* buff = NULL;
    status_t status = NO_ERROR;

    status = mDriver->getPreviewFrame(&buff);
    if(buff == NULL || status != NO_ERROR)
        return status;

    if (status == NO_ERROR) {
        buff->setOwner(this);
        buff->mType = BUFFER_TYPE_PREVIEW;
        CameraBuffer *convBuff = getFreeBuffer();
        if (convBuff == 0) {
            ALOGE("No intermediate buffers left");
            status = NO_MEMORY;
            returnBuffer(buff);
            return status;
        } else {
            status = mPipeThread->preview(buff, convBuff);
        }
    } else {
        ALOGE("Error gettting preview frame from driver");
    }
    return status;
}

status_t ControlThread::dequeueRecording()
{
    LOG2("@%s", __FUNCTION__);
    CameraBuffer* buff;
    nsecs_t timestamp;
    status_t status = NO_ERROR;

    status = mDriver->getRecordingFrame(&buff, &timestamp);

    if (status == NO_ERROR) {
        buff->setOwner(this);
        buff->mType = BUFFER_TYPE_VIDEO;

        int width, height;
        mParameters.getVideoSize(&width, &height);
        CameraBuffer *convBuff = getFreeBuffer();
        if (convBuff == 0) {
            ALOGE("No intermediate buffers left");
            status = NO_MEMORY;
            returnBuffer(buff);
            return status;
        }
        mLastRecordingBuff = buff;
        // See if recording has started.
        // If it has, process the buffer
        // If it hasn't, do preview only

        if (mState == STATE_RECORDING) {
            status = mPipeThread->previewVideo(buff, convBuff, timestamp);
        } else {
            status = mPipeThread->preview(buff, convBuff);
        }
    } else {
        ALOGE("Error: getting recording from driver\n");
    }

    return status;
}

bool ControlThread::threadLoop()
{
    LOG2("@%s", __FUNCTION__);
    status_t status = NO_ERROR;

    mThreadRunning = true;
    while (mThreadRunning) {

        switch (mState) {

        case STATE_STOPPED:
            LOG2("In STATE_STOPPED...");
            // in the stop state all we do is wait for messages
            status = waitForAndExecuteMessage();
            break;

        case STATE_PREVIEW_STILL:
            LOG2("In STATE_PREVIEW_STILL...");
            // message queue always has priority over getting data from the
            // driver no matter what state we are in
            if (!mMessageQueue.isEmpty()) {
                status = waitForAndExecuteMessage();
            } else {
                // make sure driver has data before we ask for some
                if (mDriver->dataAvailable())
                    status = dequeuePreview();
                else
                    status = waitForAndExecuteMessage();
            }
            break;

        case STATE_PREVIEW_VIDEO:
        case STATE_RECORDING:
            LOG2("In %s...", mState == STATE_PREVIEW_VIDEO ? "STATE_PREVIEW_VIDEO" : "STATE_RECORDING");
            // message queue always has priority over getting data from the
            // driver no matter what state we are in
            if (!mMessageQueue.isEmpty()) {
                status = waitForAndExecuteMessage();
            } else {

                // make sure driver has data before we ask for some
                if (mDriver->dataAvailable()) {
                    status = dequeueRecording();
                } else {
                    status = waitForAndExecuteMessage();
                }
            }
            break;

        case STATE_CAPTURE:
            LOG2("In STATE_CAPTURE...");
            // just wait until we have somthing to do
            status = waitForAndExecuteMessage();
            break;

        default:
            break;
        };
    }

    return false;
}

status_t ControlThread::requestExitAndWait()
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
