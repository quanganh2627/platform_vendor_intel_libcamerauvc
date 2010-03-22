#ifndef ANDROID_HARDWARE_INTEL_CAMERA_H
#define ANDROID_HARDWARE_INTEL_CAMERA_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ci.h"
#include "ci_adv.h"

#ifdef __cplusplus
}
#endif

namespace android {

#define SNR_NAME_LEN    50
#define RES_TEXT_LEN    20

/*Zheng*/
typedef enum {
        SENSOR_INPUT_MIPI,
        SENSOR_INPUT_PARALLEL
} sensor_input_t;

typedef struct _sensor_res {
	ci_resolution   res;
} sensor_res_t;

typedef void *  sensor_dev_t;

typedef struct _sensor_info {
        char            name[SNR_NAME_LEN];
	ci_sensor_num   snr_id;
	unsigned int    type;
	sensor_input_t  input;
        sensor_res_t    **resolutions;
        unsigned int    res_num;
} sensor_info_t;

typedef struct _ci_struct {
        int             major_version;
        int             minor_version;
        ci_context_id           context;
	ci_sensor_num           snr_id;
	ci_device_id            isp_dev;
	ci_device_id            snr_dev;
	unsigned short          snr_width;
	unsigned short          snr_height;
	unsigned short          fm_width;
	unsigned short          fm_height;
	unsigned short          continous_af;

	ci_isp_frame_id         *frames;
	unsigned int            frame_num;
	unsigned int            max_lock_frame_num;
	unsigned int            cur_frame;

	unsigned int    *buf_status;
} ci_struct_t;

/* 
 * setting value context 
 */
typedef struct _adv_setting {
	ci_jpeg_ratio	jpeg_ratio;	/* for image mode only */
	ci_ie_mode	image_effect;

	/* 3A for soc sensor */
	unsigned int	soc_snr_af_on;
	unsigned int	soc_snr_ae_on;
	unsigned int	soc_snr_awb_on;
	/* 3A for raw sensor */
	ci_isp_afss_mode	raw_snr_afss_mode;
	ci_isp_awb_mode		raw_snr_awb_mode;
	ci_isp_awb_sub_mode	raw_snr_awb_submode;
	ci_isp_aec_mode		raw_snr_aec_mode;
	
	/* flash light mode */
	int		flash_light_mode;

	/* color settings */
/*	
	unsigned char	brightness;
	unsigned char	hue;
	unsigned char	saturation;
	unsigned char	contrast;
*/
} adv_setting_t;

class IntelCamera {
public:
    IntelCamera();
    ~IntelCamera();

    void struct_init(void);
    void struct_finalize(void);
    void capture_init(unsigned int sensor_width, 
		    unsigned int sensor_height,
		    ci_frame_format frame_fmt,
		    unsigned int frame_num);
    void capture_finalize(void);
    void capture_start(void);
    void capture_map_frame(void);
    void capture_unmap_frame(void);

    unsigned int capture_grab_frame(void *buffer);
    unsigned int capture_grab_record_frame(void *buffer);
    void capture_recycle_frame(void);

    void adv_setting_init(void);

    void capture_setup_AE(unsigned int sw);
    void capture_setup_AWB(unsigned int sw);
    void capture_setup_AF(unsigned int sw);

    void auto_focus_process(void);
    void auto_exposure_process(void);
    void auto_white_balance_process(void);

    void capture_setup_jpeg_ratio(ci_jpeg_ratio ratio);
    void capture_setup_image_effect(ci_ie_mode mode);

    int is_sensor_support_resolution(int w, int h);
    void get_max_sensor_resolution(int *w, int *h);
    sensor_info_t *get_sensor_infos(void);
    void print_sensor_infos(void);

private:
    void yuv_to_rgb16(unsigned char y,unsigned char u, unsigned char v, unsigned char *rgb);
    void yuyv422_to_rgb16(unsigned char *buf, unsigned char *rgb, int width, int height);
    void yuyv422_to_yuv420(unsigned char *bufsrc, unsigned char *bufdest, int width, int height);
    void yuyv422_to_yuv420sp(unsigned char *bufsrc, unsigned char *bufdest, int width, int height);

    void alloc_sensor_infos(void);
    void free_sensor_infos(void);

    void setup_AE_for_soc_snr(void);
    void setup_AE_for_raw_snr(void);
    void setup_AWB_for_soc_snr(void);
    void setup_AWB_for_raw_snr(void);
    void setup_AF_for_soc_snr(void);
    void setup_AF_for_raw_snr(void);

    ci_struct_t ci_str;
    adv_setting_t adv_setting;
    ci_isp_frame_map_info *frame_infos;
    ci_frame_format cur_frame_fmt;

    sensor_info_t *snr_info;
};

}; // namespace android

#endif // ANDROID_HARDWARE_INTEL_CAMERA_H
