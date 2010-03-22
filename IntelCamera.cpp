#define LOG_TAG "IntelCamera"
#include <utils/Log.h>

#include "IntelCamera.h"


namespace android {

#define CHECK_RET(ret, cond, msg)					\
  if((ret) != (cond)) {							\
    LOGE("%s: %s failed error code = %d", __FUNCTION__, (msg), (ret));	\
  }									\
  else {								\
    if( DEBUG_LEVEL > 0) {						\
      LOGD("%s: %s success", __FUNCTION__, (msg));			\
    }									\
  }
#define CHECK_CI_RET(ret, msg)			\
  CHECK_RET(ret, CI_STATUS_SUCCESS, msg)


IntelCamera::IntelCamera()
  : frame_infos(0),
    cur_frame_fmt(0),
    snr_info(0)
{
  struct_init();
}

IntelCamera::~IntelCamera()
{
  struct_finalize();
}

void IntelCamera::struct_init(void)
{
  int ret;
  /* initialize ci */
  ret = ci_initialize(&ci_str.major_version, &ci_str.minor_version);
  CHECK_CI_RET(ret, "ci initialize");
  alloc_sensor_infos();
  adv_setting_init();
}

void IntelCamera::struct_finalize(void)
{
  int ret;
  free_sensor_infos();
  ret = ci_terminate();
  CHECK_CI_RET(ret, "ci terminate");
}

void IntelCamera::capture_init(unsigned int sensor_width, 
				unsigned int sensor_height,
				ci_frame_format frame_fmt,
				unsigned int frame_num)
{
  int ret;
  /* create context for view finding */
  ret = ci_create_context(&(ci_str.context));
  CHECK_CI_RET(ret, "create context");

  /* config contexts */
  ci_str.snr_id = snr_info->snr_id;
  ret = ci_context_set_cfg(ci_str.context, CI_CFG_SENSOR,
			   (void*)(&ci_str.snr_id));
  CHECK_CI_RET(ret, "set sensor");

  /* set sensor output resolution */
  ci_resolution res;
  res.width = sensor_width;
  res.height = sensor_height;
  ret = ci_context_set_cfg(ci_str.context, CI_CFG_SENSOR_RES,
			   (void *)(&res));
  CHECK_CI_RET(ret, "set sensor resolution");
  if ( ret == CI_STATUS_SUCCESS ) {
    ci_str.snr_width = res.width;
    ci_str.snr_height = res.height;
  }

  /* get isp and snr device */
  ret = ci_get_device(ci_str.context, CI_DEVICE_SENSOR,
		&(ci_str.snr_dev));
  CHECK_CI_RET(ret, "get sensor device");

  ret = ci_get_device(ci_str.context, CI_DEVICE_ISP,
		&(ci_str.isp_dev));
  CHECK_CI_RET(ret, "get isp device");

  /*start context */
  ret = ci_start_context(ci_str.context);
  CHECK_CI_RET(ret, "start context");

  /* get isp config to determine whether to use continous_af */
  ci_isp_config isp_cfg;
  ret = ci_isp_get_cfg(ci_str.isp_dev, &isp_cfg);
  ci_str.continous_af = isp_cfg.flags.continous_af;
  CHECK_CI_RET(ret, "get isp config");

  /* create frames */
  //  ci_str.frames = (ci_isp_frame_id *)malloc(sizeof(ci_isp_frame_id)*frame_num);
    ci_str.frames = new ci_isp_frame_id[frame_num];

  unsigned int w, h;
  w = sensor_width;
  h = sensor_height;
  ci_str.fm_width = w;
  ci_str.fm_height = h;
  /* VIDIOC_S_FMT, VIDIOC_REQBUFS */
  ret = ci_isp_create_frames(ci_str.isp_dev, &w, &h,
		       frame_fmt,
		       frame_num,
		       ci_str.frames);
  CHECK_CI_RET(ret, "isp create frames");
  cur_frame_fmt = frame_fmt;

  ci_str.frame_num = frame_num;
}

void IntelCamera::capture_finalize(void)
{
  int ret;
  /* destroy frames */
  ret = ci_isp_destroy_frames(ci_str.isp_dev, ci_str.frames);
  CHECK_CI_RET(ret, "destory frames");
  //free(ci_str.frames);
  delete [] ci_str.frames;
  /* stop context */
  ret = ci_stop_context(ci_str.context);
  CHECK_CI_RET(ret, "stop context");
  /* destroy context */
  ret = ci_destroy_context(ci_str.context);
  CHECK_CI_RET(ret, "destory context");
  ci_str.cur_frame = 0;
}

void IntelCamera::capture_start(void)
{
  int ret;
  ret = ci_isp_max_num_lock_frames(ci_str.isp_dev,
			     &ci_str.max_lock_frame_num);
  CHECK_CI_RET(ret, "isp max num lock frames");

  /* VIDIOC_STREAMON , VIDIOC_QUERYBUF */
  ret = ci_isp_start_capture(ci_str.isp_dev);
  CHECK_CI_RET(ret, "isp start capture");

  unsigned int i, frame_num = ci_str.frame_num;
  for (i = 0; i < frame_num; i++) {
    /* VIDIOC_QBUF */
    ret = ci_isp_set_frame_ext(ci_str.isp_dev,
			       ci_str.frames[i]);
    CHECK_CI_RET(ret, "isp set frame ext");
  }

}

void IntelCamera::capture_map_frame(void)
{
  int ret;
  unsigned int i, frame_num = ci_str.frame_num;
  frame_infos = new ci_isp_frame_map_info[frame_num];
  
  for(i = 0; i < frame_num; i++) {
    ret = ci_isp_map_frame(ci_str.isp_dev, ci_str.frames[i], &(frame_infos[i]));
    CHECK_CI_RET(ret, "capture map frame");
    LOGD("%s : frame_infos[%u]=%p",__func__, i, &(frame_infos[i]));
  }
}

void IntelCamera::capture_unmap_frame(void)
{
  int ret;
  unsigned int i, frame_num = ci_str.frame_num;

  for(i = 0; i < frame_num; i++) {
    ret = ci_isp_unmap_frame(ci_str.isp_dev, &(frame_infos[i]));
    CHECK_CI_RET(ret, "capture unmap frame");
    LOGD("%s : frame_infos[%u]=%p",__func__, i, &(frame_infos[i]));
  }
  delete [] frame_infos;
}

unsigned int IntelCamera::capture_grab_frame(void *buffer)
{
  int ret;
  unsigned int frame;

  /* VIDIOC_DQBUF */
  ret = ci_isp_capture_frame_ext(ci_str.isp_dev, (ci_isp_frame_id *)&frame);
  CHECK_CI_RET(ret, "capture frame ext");
  ci_str.cur_frame = frame;
  /*
  LOGD("frame_infos[%u].addr=0x%p, frame_infos[%u].size=0x%x, buffer=0x%p",
       frame,
       frame_infos[frame].addr,
       frame,
       frame_infos[frame].size,
       buffer);
  */
  if(buffer != NULL) {
    switch(cur_frame_fmt) {
    case INTEL_PIX_FMT_YUYV :
      //      LOGD("INTEL_PIX_FMT_YUYV");
      yuyv422_to_yuv420sp((unsigned char *)(frame_infos[frame]).addr,
			  (unsigned char *)buffer,
			  ci_str.fm_width,
			  ci_str.fm_height);
      break;
    case INTEL_PIX_FMT_NV12 :
      //      LOGD("INTEL_PIX_FMT_NV12");
      memcpy(buffer, frame_infos[frame].addr, frame_infos[frame].size);
      break;
    case INTEL_PIX_FMT_JPEG :
      //      LOGD("INTEL_PIX_FMT_JPEG");
      memcpy(buffer, frame_infos[frame].addr, frame_infos[frame].size);
      break;
    default :
      LOGE("Unknown Format type");
    }
  }
  ret = ci_bp_correct(ci_str.context);
  CHECK_CI_RET(ret, "bp correct");
  ret = ci_bl_compensate(ci_str.context);
  CHECK_CI_RET(ret, "bl compensate");

  return frame;
}

unsigned int IntelCamera::capture_grab_record_frame(void *buffer)
{
  if(buffer != NULL) {
    switch(cur_frame_fmt) {
    case INTEL_PIX_FMT_YUYV :
      yuyv422_to_yuv420sp((unsigned char *)(frame_infos[ci_str.cur_frame]).addr,
			  (unsigned char *)buffer,
			  ci_str.fm_width,
			  ci_str.fm_height);
      break;
    case INTEL_PIX_FMT_NV12 :
      memcpy(buffer, frame_infos[ci_str.cur_frame].addr, frame_infos[ci_str.cur_frame].size);
      break;
    default :
      LOGE("Unknown Format type");
    }
  }
  return ci_str.cur_frame;
}

void IntelCamera::capture_recycle_frame(void)
{
  int ret;
  //  ci_str.cur_frame = (ci_str.cur_frame + 1) % ci_str.frame_num;
  ret = ci_isp_set_frame_ext(ci_str.isp_dev,ci_str.frames[ci_str.cur_frame]);
  CHECK_CI_RET(ret, "isp set frame ext");
}

void IntelCamera::capture_setup_AE(unsigned int sw)
{
  if ( snr_info->type == SENSOR_TYPE_2M ) {
    setup_AE_for_soc_snr();
  } else {
    setup_AE_for_raw_snr();
  }
}

void IntelCamera::capture_setup_AWB(unsigned int sw)
{
  if ( snr_info->type == SENSOR_TYPE_2M ) {
    setup_AWB_for_soc_snr();
  } else {
    setup_AWB_for_raw_snr();
  }
}

void IntelCamera::capture_setup_AF(unsigned int sw)
{
  if ( snr_info->type == SENSOR_TYPE_2M ) {
    setup_AF_for_soc_snr();
  } else {
    setup_AF_for_raw_snr();
  }
}

void IntelCamera::auto_focus_process(void)
{
  int ret;
  if ( snr_info->type == SENSOR_TYPE_5M ) {
      ret = ci_af_process(ci_str.context);
      CHECK_CI_RET(ret, "ci af process");
  }
}

void IntelCamera::auto_exposure_process(void)
{
  int ret;
  if ( snr_info->type == SENSOR_TYPE_5M ) {
      ret = ci_ae_process(ci_str.context);
      CHECK_CI_RET(ret, "ci ae process");
  }
}

void IntelCamera::auto_white_balance_process(void)
{
  int ret;
   if ( snr_info->type == SENSOR_TYPE_5M ) {
       ret = ci_awb_process(ci_str.context);
       CHECK_CI_RET(ret, "ci awb process");
   }
}

void IntelCamera::capture_setup_jpeg_ratio(ci_jpeg_ratio ratio)
{
  /*
typedef enum
  {
    CI_JPEG_HIGH_COMPRESSION,
    CI_JPEG_LOW_COMPRESSION,
    CI_JPEG_01_PERCENTAGE,
    CI_JPEG_20_PERCENTAGE,
    CI_JPEG_30_PERCENTAGE,
    CI_JPEG_40_PERCENTAGE,
    CI_JPEG_50_PERCENTAGE,
    CI_JPEG_60_PERCENTAGE,
    CI_JPEG_70_PERCENTAGE,
    CI_JPEG_80_PERCENTAGE,
    CI_JPEG_90_PERCENTAGE,
        CI_JPEG_99_PERCENTAGE
  }ci_jpeg_ratio;
  */
  int ret;
  /* set image effect */
  adv_setting.jpeg_ratio = ratio;
  ret = ci_context_set_cfg(ci_str.context, CI_CFG_JPEG,
			   (void*)&(adv_setting.jpeg_ratio));
  CHECK_CI_RET(ret, "set jpeg ratio");
}

void IntelCamera::capture_setup_image_effect(ci_ie_mode mode)
{

  /*
  <typedef for enum of image effect mode for context config>
  typedef enum {
    <no image effect (bypass)>
    CI_IE_MODE_OFF = 0,
    <Set a fixed chrominance of 128 (neutral grey)>
    CI_IE_MODE_GRAYSCALE,
    <Luminance and chrominance data is being inverted>
    CI_IE_MODE_NEGATIVE,
    <Chrominance is changed to produce a historical like brownish image color>
    CI_IE_MODE_SEPIA,
    <Converting picture to grayscale while maintaining one color component>
    CI_IE_MODE_COLORSEL,
    <Edge detection, will look like an relief made of metal>
    CI_IE_MODE_EMBOSS,
    <Edge detection, will look like a pencil drawing>
    CI_IE_MODE_SKETCH
  }ci_ie_mode;
  */
  int ret;
  /* set image effect */
  adv_setting.image_effect = mode;
  ret = ci_context_set_cfg(ci_str.context, CI_CFG_IE,
			   (void*)&(adv_setting.image_effect));
  CHECK_CI_RET(ret, "set image effect");
}

void IntelCamera::setup_AE_for_soc_snr(void)
{
  int ret;
  adv_setting.soc_snr_ae_on = ON;

  if ( adv_setting.soc_snr_ae_on  == (unsigned int)ON ) {
    LOGD("set AE ON");
  } else {
    LOGD("set AE OFF");
  }
  /* setup AE */
  ret = ci_context_set_cfg(ci_str.context, CI_CFG_AE,
			   (void*)&(adv_setting.soc_snr_ae_on));
  CHECK_CI_RET(ret, "set config for AE");
}

void IntelCamera::setup_AE_for_raw_snr(void)
{
    int ret;
    ci_ae_param ae_param = {
      {3, 1, 516, 388},
      {0, 0, 640, 480}};

    adv_setting.raw_snr_aec_mode = CI_ISP_AEC_INTEGRAL;
    LOGD("ae_param.meas_wnd: hoffs = %d, voffs = %d, ", ae_param.meas_wnd.hoffs, ae_param.meas_wnd.voffs);
    LOGD("w = %d, h = %d", ae_param.meas_wnd.hsize, ae_param.meas_wnd.vsize);
    LOGD("ae_param.hist_wnd: hoffs = %d, voffs = %d, ", ae_param.hist_wnd.hoffs, ae_param.hist_wnd.voffs);
    LOGD("w = %d, h = %d", ae_param.hist_wnd.hsize, ae_param.hist_wnd.vsize);
    if (adv_setting.raw_snr_aec_mode == CI_ISP_AEC_INTEGRAL) {
      LOGD("set AE ON");
      ret = ci_ae_setup(ci_str.context, ae_param, ON);
    } else {
      LOGD("set AE OFF");
      ret = ci_ae_setup(ci_str.context, ae_param, OFF);
    }
    CHECK_CI_RET(ret, "ci_ae_setup");
}

void IntelCamera::setup_AWB_for_soc_snr(void)
{
  int ret;
  if ( adv_setting.soc_snr_awb_on == (unsigned int)ON ) {
    LOGD("set AWB ON");
  } else {
    LOGD("set AWB OFF");
  }
  /* setup AWB */
  ret = ci_context_set_cfg(ci_str.context, CI_CFG_AWB,
			   (void*)&(adv_setting.soc_snr_awb_on));
  CHECK_CI_RET(ret, "set config for AWB");
}

void IntelCamera::setup_AWB_for_raw_snr(void)
{
  int ret;
  ci_wb_param wb_param = {
    {0, 0, 640, 480},
    CI_ISP_AWB_AUTO,
    CI_ISP_AWB_MAN_CIE_D65};

  LOGD("wb_param.window: hoffs = %d, voffs = %d, ", wb_param.window.hoffs, wb_param.window.voffs);
  LOGD("w = %d, h = %d", wb_param.window.hsize, wb_param.window.vsize);

  if (wb_param.mode == CI_ISP_AWB_COMPLETELY_OFF) {
    LOGD("set AWB OFF: %d, %d", (int)wb_param.mode, (int)wb_param.sub_mode);
    ret = ci_wb_setup(ci_str.context, wb_param, OFF);
  } else {
    LOGD("set AWB OFF: %d, %d", (int)wb_param.mode, (int)wb_param.sub_mode);
    ret = ci_wb_setup(ci_str.context, wb_param, ON);
  }
  CHECK_CI_RET(ret, "ci_awb_setup");
}

void IntelCamera::setup_AF_for_soc_snr(void)
{
  int ret;
  if (adv_setting.soc_snr_af_on == (unsigned int)ON ) {
    LOGD("set AF ON");
  } else {
    LOGD("set AF OFF");
  }

  /* setup AF */
  ret = ci_context_set_cfg(ci_str.context, CI_CFG_AF,
			   (void *)&(adv_setting.soc_snr_af_on));
  CHECK_CI_RET(ret, "set config for AF");
}

void IntelCamera::setup_AF_for_raw_snr(void)
{
  int ret;
  ci_af_param af_param = {
    {((640)/2-(50)/2),((480)/2-(50)/2),50,50},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    CI_ISP_AFSS_OFF};

  if(af_param.mode == CI_ISP_AFSS_OFF) {
    LOGD("set AF OFF: %d",af_param.mode);
    ret = ci_af_setup(ci_str.context, af_param, OFF);
  } else {
    LOGD("set AF OFF: %d",af_param.mode);
    ret = ci_af_setup(ci_str.context, af_param, ON);
  }

  CHECK_CI_RET(ret, "ci_af_setup");
}

void IntelCamera::adv_setting_init(void)
{
  adv_setting.soc_snr_af_on = OFF;
  adv_setting.soc_snr_ae_on = ON;
  adv_setting.soc_snr_awb_on = ON;
  adv_setting.raw_snr_afss_mode = CI_ISP_AFSS_OFF;
  adv_setting.raw_snr_awb_mode = CI_ISP_AWB_AUTO;
  adv_setting.raw_snr_awb_submode = CI_ISP_AWB_MAN_CIE_D65;
  adv_setting.raw_snr_aec_mode = CI_ISP_AEC_INTEGRAL;

  adv_setting.jpeg_ratio = CI_JPEG_HIGH_COMPRESSION;
  adv_setting.image_effect = CI_IE_MODE_OFF;

  adv_setting.flash_light_mode = OFF;
  /* color setting */
}

void IntelCamera::alloc_sensor_infos(void)
{
  ci_context_id ctx;
  int snr_id;
  int ret;

  ret = ci_create_context(&ctx);
  CHECK_CI_RET(ret, "ci_create_context"); 

  for(snr_id = CI_SENSOR_SOC_0; snr_id <= CI_SENSOR_RAW_1; snr_id++) {

    ret = ci_context_set_cfg(ctx, CI_CFG_SENSOR, (void *)&snr_id);

    if(ret != CI_STATUS_SUCCESS) {
      snr_info = NULL;
      continue;
    } else {
      ci_device_id snr_dev;
      ci_sensor_caps snr_cap;
      ci_resolution ress[CI_MAX_RES_NUM];
      int res_num, k;
      
      snr_info = new sensor_info_t;
      ret = ci_get_device(ctx, CI_DEVICE_SENSOR, &snr_dev);
      CHECK_CI_RET(ret, "ci get sensor device");
      
      ret = ci_sensor_get_caps(snr_dev, &snr_cap);
      CHECK_CI_RET(ret, "ci get isp device");
      
      ret = ci_get_resolution((ci_sensor_num)snr_id, ress, &res_num, INTEL_PIX_FMT_JPEG);
      CHECK_CI_RET(ret, "ci_get_resolution");
      snr_info->res_num = res_num;
      
      strncpy(snr_info->name, snr_cap.name, SNR_NAME_LEN-1);
      
      LOGD("Found sensor: %s\n", snr_cap.name);
      
      if ((ret = strcmp(snr_cap.name, "s5k4e1")) == 0) {
	snr_info->input = SENSOR_INPUT_MIPI;
	LOGD("It is a MIPI sensor, auto-review not supported");
      } else {
	snr_info->input = SENSOR_INPUT_PARALLEL;
      }
      
      strncpy(snr_info->name, snr_cap.name, SNR_NAME_LEN-1);    
      snr_info->snr_id = (ci_sensor_num)snr_id;
      
      if(snr_id == CI_SENSOR_SOC_0 || snr_id == CI_SENSOR_SOC_1)
	snr_info->type = SENSOR_TYPE_2M;
      else
	snr_info->type = SENSOR_TYPE_5M;
      
      snr_info->resolutions = new sensor_res_t*[res_num];
      
      for(k = 0; k < res_num; k++) {
	snr_info->resolutions[k] = new sensor_res_t;
	snr_info->resolutions[k]->res.width = ress[k].width;
	snr_info->resolutions[k]->res.height = ress[k].height;
	snr_info->resolutions[k]->res.fps = ress[k].fps;
      }
      break;
    }

  }

  ret = ci_destroy_context(ctx);
  CHECK_CI_RET(ret, "ci_destroy_context");
}

void IntelCamera::free_sensor_infos(void)
{
   if (snr_info != NULL) {
     for(unsigned int k = 0; k < snr_info->res_num; k++) {
       delete snr_info->resolutions[k];
     }
     delete [] snr_info->resolutions;
     delete snr_info;
  }
}

int IntelCamera::is_sensor_support_resolution(int w, int h)
{
  sensor_res_t *snr_res;

  if (snr_info != NULL) {
      LOGD("res width=%d, height=%d",w,h);
      for(unsigned int i = 0; i < snr_info->res_num; i++) {
	snr_res = snr_info->resolutions[i];
	LOGD("snr_info width=%d, height=%d",snr_res->res.width,snr_res->res.height);
	if((unsigned int)w == snr_res->res.width &&
	   (unsigned int)h == snr_res->res.height) {
	  return 1;
	}
      }
  }
  return 0;
}

void IntelCamera::get_max_sensor_resolution(int *w, int *h)
{
    if (snr_info != NULL) {
      sensor_res_t *snr_res =
	snr_info->resolutions[snr_info->res_num - 1];
      *w = snr_res->res.width;
      *h = snr_res->res.height;
    }
}

sensor_info_t * IntelCamera::get_sensor_infos(void)
{
  return snr_info;
}

void IntelCamera::print_sensor_infos(void)
{
    if (snr_info != NULL) {
      LOGD("Current Sensor Name: %s", snr_info->name);
      LOGD("Type: %s", snr_info->type == SENSOR_TYPE_2M ?  "SOC(2M)" : "RAW(5M)");
      LOGD("Supported Jpeg Resolutions: ");
      for(unsigned int i = 0; i < snr_info->res_num; i++) {
	sensor_res_t *snr_res = snr_info->resolutions[i];
	LOGD("\t %d*%d", snr_res->res.width, snr_res->res.height);
      }
    }
}
/*
void IntelCamera::nv12_to_nv21(unsigned char *bufsrc, unsigned char *bufdest, int width, int height)
{
  unsigned char *y;
}
*/
void IntelCamera::yuv_to_rgb16(unsigned char y,unsigned char u, unsigned char v, unsigned char *rgb)
{
	register int r,g,b;
	int rgb16;

	r = (1192 * (y - 16) + 1634 * (v - 128) ) >> 10;
	g = (1192 * (y - 16) - 833 * (v - 128) - 400 * (u -128) ) >> 10;
	b = (1192 * (y - 16) + 2066 * (u - 128) ) >> 10;

	r = r > 255 ? 255 : r < 0 ? 0 : r;
	g = g > 255 ? 255 : g < 0 ? 0 : g;
	b = b > 255 ? 255 : b < 0 ? 0 : b;

	rgb16 = (int)(((r >> 3)<<11) | ((g >> 2) << 5)| ((b >> 3) << 0));

	*rgb = (unsigned char)(rgb16 & 0xFF);
	rgb++;
	*rgb = (unsigned char)((rgb16 & 0xFF00) >> 8);
}

void IntelCamera::yuyv422_to_rgb16(unsigned char *buf, unsigned char *rgb, int width, int height)
{
	int x,y,z=0;
	int blocks;

	blocks = (width * height) * 2;
	for (y = 0; y < blocks; y+=4) {
		unsigned char Y1, Y2, U, V;

		Y1 = buf[y + 0];
		U = buf[y + 1];
		Y2 = buf[y + 2];
		V = buf[y + 3];

		yuv_to_rgb16(Y1, U, V, &rgb[y]);
		yuv_to_rgb16(Y2, U, V, &rgb[y + 2]);
	}
}

void IntelCamera::yuyv422_to_yuv420(unsigned char *bufsrc, unsigned char *bufdest, int width, int height)
{
	unsigned char *ptrsrcy1, *ptrsrcy2;
	unsigned char *ptrsrcy3, *ptrsrcy4;
	unsigned char *ptrsrccb1, *ptrsrccb2;
	unsigned char *ptrsrccb3, *ptrsrccb4;
	unsigned char *ptrsrccr1, *ptrsrccr2;
	unsigned char *ptrsrccr3, *ptrsrccr4;
	int srcystride, srcccstride;

	ptrsrcy1  = bufsrc ;
	ptrsrcy2  = bufsrc + (width<<1) ;
	ptrsrcy3  = bufsrc + (width<<1)*2 ;
	ptrsrcy4  = bufsrc + (width<<1)*3 ;

	ptrsrccb1 = bufsrc + 1;
	ptrsrccb2 = bufsrc + (width<<1) + 1;
	ptrsrccb3 = bufsrc + (width<<1)*2 + 1;
	ptrsrccb4 = bufsrc + (width<<1)*3 + 1;

	ptrsrccr1 = bufsrc + 3;
	ptrsrccr2 = bufsrc + (width<<1) + 3;
	ptrsrccr3 = bufsrc + (width<<1)*2 + 3;
	ptrsrccr4 = bufsrc + (width<<1)*3 + 3;

	srcystride  = (width<<1)*3;
	srcccstride = (width<<1)*3;

	unsigned char *ptrdesty1, *ptrdesty2;
	unsigned char *ptrdesty3, *ptrdesty4;
	unsigned char *ptrdestcb1, *ptrdestcb2;
	unsigned char *ptrdestcr1, *ptrdestcr2;
	int destystride, destccstride;

	ptrdesty1 = bufdest;
	ptrdesty2 = bufdest + width;
	ptrdesty3 = bufdest + width*2;
	ptrdesty4 = bufdest + width*3;

	ptrdestcb1 = bufdest + width*height;
	ptrdestcb2 = bufdest + width*height + (width>>1);

	ptrdestcr1 = bufdest + width*height + ((width*height) >> 2);
	ptrdestcr2 = bufdest + width*height + ((width*height) >> 2) + (width>>1);

	destystride  = (width)*3;
	destccstride = (width>>1);

	int i, j;

	for(j=0; j<(height/4); j++) {
		for(i=0;i<(width/2);i++) {
			(*ptrdesty1++) = (*ptrsrcy1);
			(*ptrdesty2++) = (*ptrsrcy2);
			(*ptrdesty3++) = (*ptrsrcy3);
			(*ptrdesty4++) = (*ptrsrcy4);

			ptrsrcy1 += 2;
			ptrsrcy2 += 2;
			ptrsrcy3 += 2;
			ptrsrcy4 += 2;

			(*ptrdesty1++) = (*ptrsrcy1);
			(*ptrdesty2++) = (*ptrsrcy2);
			(*ptrdesty3++) = (*ptrsrcy3);
			(*ptrdesty4++) = (*ptrsrcy4);
			ptrsrcy1 += 2;
			ptrsrcy2 += 2;
			ptrsrcy3 += 2;
			ptrsrcy4 += 2;

			(*ptrdestcb1++) = (*ptrsrccb1);
			(*ptrdestcb2++) = (*ptrsrccb3);
			ptrsrccb1 += 4;
			ptrsrccb3 += 4;

			(*ptrdestcr1++) = (*ptrsrccr1);
			(*ptrdestcr2++) = (*ptrsrccr3);

			ptrsrccr1 += 4;
			ptrsrccr3 += 4;
		}

		/* Update src pointers */
		ptrsrcy1  += srcystride;
		ptrsrcy2  += srcystride;
		ptrsrcy3  += srcystride;
		ptrsrcy4  += srcystride;

		ptrsrccb1 += srcccstride;
		ptrsrccb3 += srcccstride;

		ptrsrccr1 += srcccstride;
		ptrsrccr3 += srcccstride;

		/* Update dest pointers */
		ptrdesty1 += destystride;
		ptrdesty2 += destystride;
		ptrdesty3 += destystride;
		ptrdesty4 += destystride;

		ptrdestcb1 += destccstride;
		ptrdestcb2 += destccstride;

		ptrdestcr1 += destccstride;
		ptrdestcr2 += destccstride;
	}
}

void IntelCamera::yuyv422_to_yuv420sp(unsigned char *bufsrc, unsigned char *bufdest, int width, int height)
{
	unsigned char *ptrsrcy1, *ptrsrcy2;
	unsigned char *ptrsrcy3, *ptrsrcy4;
	unsigned char *ptrsrccb1, *ptrsrccb2;
	unsigned char *ptrsrccb3, *ptrsrccb4;
	unsigned char *ptrsrccr1, *ptrsrccr2;
	unsigned char *ptrsrccr3, *ptrsrccr4;
	int srcystride, srcccstride;

	ptrsrcy1  = bufsrc ;
	ptrsrcy2  = bufsrc + (width<<1) ;
	ptrsrcy3  = bufsrc + (width<<1)*2 ;
	ptrsrcy4  = bufsrc + (width<<1)*3 ;

	ptrsrccb1 = bufsrc + 1;
	ptrsrccb2 = bufsrc + (width<<1) + 1;
	ptrsrccb3 = bufsrc + (width<<1)*2 + 1;
	ptrsrccb4 = bufsrc + (width<<1)*3 + 1;

	ptrsrccr1 = bufsrc + 3;
	ptrsrccr2 = bufsrc + (width<<1) + 3;
	ptrsrccr3 = bufsrc + (width<<1)*2 + 3;
	ptrsrccr4 = bufsrc + (width<<1)*3 + 3;

	srcystride  = (width<<1)*3;
	srcccstride = (width<<1)*3;

	unsigned char *ptrdesty1, *ptrdesty2;
	unsigned char *ptrdesty3, *ptrdesty4;
	unsigned char *ptrdestcb1, *ptrdestcb2;
	unsigned char *ptrdestcr1, *ptrdestcr2;
	int destystride, destccstride;

	ptrdesty1 = bufdest;
	ptrdesty2 = bufdest + width;
	ptrdesty3 = bufdest + width*2;
	ptrdesty4 = bufdest + width*3;

	ptrdestcb1 = bufdest + width*height;
	ptrdestcb2 = bufdest + width*height + width;

	ptrdestcr1 = bufdest + width*height + 1;
	ptrdestcr2 = bufdest + width*height + width + 1;

	destystride  = (width)*3;
	destccstride = width;

	int i, j;

	for(j=0; j<(height/4); j++) {
		for(i=0;i<(width/2);i++) {
			(*ptrdesty1++) = (*ptrsrcy1);
			(*ptrdesty2++) = (*ptrsrcy2);
			(*ptrdesty3++) = (*ptrsrcy3);
			(*ptrdesty4++) = (*ptrsrcy4);

			ptrsrcy1 += 2;
			ptrsrcy2 += 2;
			ptrsrcy3 += 2;
			ptrsrcy4 += 2;

			(*ptrdesty1++) = (*ptrsrcy1);
			(*ptrdesty2++) = (*ptrsrcy2);
			(*ptrdesty3++) = (*ptrsrcy3);
			(*ptrdesty4++) = (*ptrsrcy4);

			ptrsrcy1 += 2;
			ptrsrcy2 += 2;
			ptrsrcy3 += 2;
			ptrsrcy4 += 2;

			(*ptrdestcb1) = (*ptrsrccb1);
			(*ptrdestcb2) = (*ptrsrccb3);
			ptrdestcb1 += 2;
			ptrdestcb2 += 2;

			ptrsrccb1 += 4;
			ptrsrccb3 += 4;

			(*ptrdestcr1) = (*ptrsrccr1);
			(*ptrdestcr2) = (*ptrsrccr3);
			ptrdestcr1 += 2;
			ptrdestcr2 += 2;

			ptrsrccr1 += 4;
			ptrsrccr3 += 4;
		}

		/* Update src pointers */
		ptrsrcy1  += srcystride;
		ptrsrcy2  += srcystride;
		ptrsrcy3  += srcystride;
		ptrsrcy4  += srcystride;

		ptrsrccb1 += srcccstride;
		ptrsrccb3 += srcccstride;

		ptrsrccr1 += srcccstride;
		ptrsrccr3 += srcccstride;

		/* Update dest pointers */
		ptrdesty1 += destystride;
		ptrdesty2 += destystride;
		ptrdesty3 += destystride;
		ptrdesty4 += destystride;

		ptrdestcb1 += destccstride;
		ptrdestcb2 += destccstride;

		ptrdestcr1 += destccstride;
		ptrdestcr2 += destccstride;
	}
}

}; // namespace android
