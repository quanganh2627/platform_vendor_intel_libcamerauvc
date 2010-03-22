/*
 * Copyright (c) 2009-2010 Wind River Systems, Inc.
 *
 * The right to copy, distribute, modify, or otherwise make use
 * of this software may be licensed only pursuant to the terms
 * of an applicable Wind River license agreement.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/fb.h>

#include <cutils/log.h>

#include "ci.h"
#include "ci_adv.h"

#define PREVIEW_WIDTH    640
#define PREVIEW_HEIGHT    480

typedef struct _adv_setting {
	ci_jpeg_ratio           jpeg_ratio;
	ci_ie_mode              image_effect;
	/* 3A for soc sensor */
	unsigned int            soc_snr_af_on;
	unsigned int            soc_snr_ae_on;
	unsigned int            soc_snr_awb_on;
	/* 3A for raw sensor */
	ci_isp_afss_mode        raw_snr_afss_mode;
	ci_isp_awb_mode         raw_snr_awb_mode;
	ci_isp_awb_sub_mode     raw_snr_awb_submode;
	ci_isp_aec_mode         raw_snr_aec_mode;
	/* flash light mode */
	int                     flash_light_mode;
} adv_setting_t;

static adv_setting_t adv_setting;

#define FB_MMAP_SIZE (PREVIEW_WIDTH * PREVIEW_HEIGHT * 2)

struct fb_fix_screeninfo fix;
struct fb_var_screeninfo var;
unsigned long *fb_buf;
static int pfbd= -1;

#define MIN(x,y)(((x)>(y))?(y):(x))
#define MAX(x,y)(((x)<(y))?(y):(x))

/* fb support */
static void fb_draw(char *buf)
{
	int fb_y = 0, fb_x = 0, cam_x = 0, cam_y = 0, cur_y = 0;
	int fb_line_length = 0, rgb_line_length = 0;
	unsigned char *buf_p = NULL, *fb_p = NULL;
	struct fb_var_screeninfo vinfo;

	cam_x = MAX(PREVIEW_WIDTH, PREVIEW_HEIGHT);
	cam_y = MIN(PREVIEW_WIDTH, PREVIEW_HEIGHT);

	fb_y = PREVIEW_WIDTH;
	fb_x = PREVIEW_HEIGHT;

	fb_line_length = fb_x * 2;
	rgb_line_length = cam_x * 2;

	if(ioctl(pfbd, FBIOGET_VSCREENINFO, &vinfo) < 0) {
		perror("ioctl FBIOGET_VSCREENINFO");
		return;
	}
	//printf("fb_mem=0x%08x, buf=0x%08x\n", fb_mem, buf);
	for (cur_y = 0; cur_y < fb_y; cur_y++) {
		//printf("curr_y=%d\n", cur_y);
		//printf("fb_line_length=%d\n", fb_line_length);
		fb_p = fb_buf + (cur_y * fb_line_length);
		buf_p = (unsigned char *)buf +
			(cur_y * rgb_line_length);
		//printf("fb_p=0x%08x buf_p=0x%08x\n", fb_p, buf_p);

		memcpy(fb_p, buf_p, fb_line_length);
	}
}

static int fb_open()
{
	pfbd = open("/dev/graphics/fb0", O_RDWR);
	if (pfbd < 0) {
		LOGE("/dev/graphics/fb0");
		goto err_fb_init_open;
	}

	if (-1 == ioctl(pfbd, FBIOGET_VSCREENINFO, &var)) {
		LOGE("ioctl FBIOGET_VSCREENINFO");
		goto err_fb_init_ioctl;
	}
	if (-1 == ioctl(pfbd, FBIOGET_FSCREENINFO, &fix)) {
		LOGE("ioctl FBIOGET_FSCREENINFO");
		goto err_fb_init_ioctl;
	}

	LOGD("<app> %s() FB_MMAP_SIZE: %d\n", __func__, FB_MMAP_SIZE);

	fb_buf = (char *)mmap(0, FB_MMAP_SIZE,
			PROT_READ | PROT_WRITE, MAP_SHARED, pfbd, 0);

	if (((int)fb_buf) == -1) {
		LOGE("mmap(fb)");
		goto err_fb_init_mmap;
	}
	LOGD("fb_buf=0x%08x\n", fb_buf);
	memset(fb_buf, 0xFF, FB_MMAP_SIZE);

	return 0;

err_fb_init_mmap:
err_fb_init_ioctl:
	close(pfbd);
	pfbd = -1;
err_fb_init_open:
	exit(0);
	return -1;
}

void adv_setting_init(void)
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
}

void dump_image(unsigned char *src, int frame, int size)
{
    char filename[64];
    int fd;

    LOGD("%d, src = %p, size = %d\n", frame, src, size);

    sprintf(filename, "/cache/preview-%02d.yuv422_packed_yuyv", frame);
    fd = open(filename, O_RDWR|O_CREAT);
    if (fd < 0) {
        LOGE("%s open failed", filename);
        return;
    }

    write(fd, src, size);

    close(fd);
}

void yuv_to_rgb16(unsigned char y,unsigned char u, unsigned char v, unsigned char *rgb)
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

void yuyv422_to_rgb16(unsigned char *buf, unsigned char *rgb, int width, int height)
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


    ci_isp_frame_id     *frames;
    ci_isp_frame_map_info *frame_infos;

int main(int argc, char *argv[])
{
    int major, minor;
    adv_setting_t       adv_setting;
    unsigned int        max_lock_frame_num;
    ci_sensor_caps      snr_cap;
    ci_device_id        snr_dev;
    ci_sensor_num       snr_id;
    ci_device_id        isp_dev;
    ci_frame_format     fmt;
    ci_resolution       res;
    ci_context_id       context;

    int                 res_num;
//    int          num_fmt;

    ci_isp_config       isp_cfg;
    ci_isp_frame_map_info jpeg_frame_info;

    int preview_width = PREVIEW_WIDTH;
    int preview_height = PREVIEW_HEIGHT;

    int                 mPreviewFrameCount=3;
    int                 mCurrentPreviewFrame=0;
    int i;

    char *mPreviewHeap;
    ci_initialize(&major, &minor);
    
    adv_setting_init();
    
    /* create context for view finding */
    ci_create_context(&context);
    
    /* config contexts */
    snr_id = CI_SENSOR_SOC_0;
    ci_context_set_cfg(context, CI_CFG_SENSOR,(void *)(&snr_id));
    
    /* set sensor output resolution */
    res.width = preview_width;
    res.height = preview_height;
    ci_context_set_cfg(context, CI_CFG_SENSOR_RES, (void *)(&res));
    
    /* get isp and snr device */
    ci_get_device(context, CI_DEVICE_SENSOR, &snr_dev);
    ci_get_device(context, CI_DEVICE_ISP, &isp_dev);

    /*start preview context */
    ci_start_context(context);
    
    /* get isp config to determine whether to use continous_af */
    ci_isp_get_cfg(isp_dev, &isp_cfg);
    LOGD("get isp config isp_cfg.flags.continuous_af=%d",isp_cfg.flags.continous_af);
    
    /* create frames used for preview */
    frames = malloc(sizeof(ci_isp_frame_id)*mPreviewFrameCount);

    /* VIDIOC_S_FMT, VIDIOC_REQBUFS */
    ci_isp_create_frames(isp_dev,
		    (unsigned int *)&preview_width,
		    (unsigned int *)&preview_height,
		    //INTEL_PIX_FMT_YUYV,
		    INTEL_PIX_FMT_NV12,
		    mPreviewFrameCount,
		    frames);
    for(i=0; i<3; i++)
	    LOGD("frames[%d]=%p, %d, size=%d",i,&(frames[i]),frames[i], sizeof(ci_isp_frame_id)*mPreviewFrameCount);
    
    ci_isp_max_num_lock_frames(isp_dev, &max_lock_frame_num);

    /* VIDIOC_STREAMON */
    ci_isp_start_capture(isp_dev);

    /*
     *   VIDIOC_QUERYBUF
     * Map a frame into client address space for access.
     */
    frame_infos = malloc(sizeof(ci_isp_frame_map_info)*mPreviewFrameCount);
    for(i=0; i< mPreviewFrameCount; i++) {
	    LOGD("frame_infos[%d]=%p , size=%d",i,&(frame_infos[i]),sizeof(ci_isp_frame_map_info)*mPreviewFrameCount);
	    ci_isp_map_frame(isp_dev, frames[i], &(frame_infos[i]));
    }
    for(i=0; i<mPreviewFrameCount; i++) {
	    /* VIDIOC_QBUF */
	    ci_isp_set_frame_ext(isp_dev, frames[i]);
    }
  
    /* setup AE */
    ci_context_set_cfg(context, CI_CFG_AE, (void*)&(adv_setting.soc_snr_ae_on));
    
    /* setup AWB */
    ci_context_set_cfg(context, CI_CFG_AWB, (void*)&(adv_setting.soc_snr_awb_on));
    
    /* setup AF */
    ci_context_set_cfg(context, CI_CFG_AF, (void *)&(adv_setting.soc_snr_af_on));
    
    /* set image effect */
    ci_context_set_cfg(context, CI_CFG_IE, (void*)&(adv_setting.image_effect));
    
    fb_open();

    mPreviewHeap =(char *)malloc(PREVIEW_WIDTH * PREVIEW_HEIGHT * 2);

    while (1) {
	    /* VIDIOC_DQBUF */
	    ci_isp_capture_frames(isp_dev, 1);
	    /* do bp_correct and bl compensate */
	    ci_bp_correct(context);
	    ci_bl_compensate(context);
	    
	    yuyv422_to_rgb16((unsigned char *)(frame_infos[mCurrentPreviewFrame]).addr,
			    (unsigned char *)mPreviewHeap, preview_width, preview_height);
	    
	    LOGD("frame_infos[%d]).addr=%p, mPreviewHeap=%p",mCurrentPreviewFrame,frame_infos[mCurrentPreviewFrame].addr,mPreviewHeap);
	    fb_draw((unsigned long*)mPreviewHeap);
		    //memcpy(mPreviewHeap->getBase(),frame_infos[mCurrentPreviewFrame].addr, mPreviewFrameSize);
	    //mDataCb(CAMERA_MSG_PREVIEW_FRAME,mPreviewBuffer, mCallbackCookie);
	    mCurrentPreviewFrame = (mCurrentPreviewFrame + 1) % mPreviewFrameCount;
	    /* VIDIOC_QBUF */
	    ci_isp_set_frame_ext(isp_dev,frames[mCurrentPreviewFrame]);
    }
    
    for (i=0; i < mPreviewFrameCount; i++) {
	    ci_isp_unmap_frame(isp_dev, &(frame_infos[i]));
    }
    free(frame_infos);
    ci_isp_destroy_frames(isp_dev, frames);
    free(frames);
    ci_stop_context(context);
    ci_destroy_context(context);
    ci_terminate();
    
    return 0;
}
