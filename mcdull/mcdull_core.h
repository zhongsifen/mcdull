// mcdull_core.h

#ifndef _mcdull_encode_
#define _mcdull_encode_ 1

#ifdef __cplusplus
extern "C" {
#endif

#include "mcdull.h"
#include "mcdull_x264.h"
#include "mcdull_dull.h"

#if mcdull_SWATAW
#include "swataw.h"
#endif

typedef struct
{
	int32_t width;
	int32_t height;

	int32_t qp;
	int32_t keyint;
	int32_t bframe;

	int32_t frame_inp;
    int32_t frame_out;

	x264_t* x264;
	x264_param_t param[1];
    x264_picture_t pic[1];
    x264_picture_t pic_out[1];
    x264_nal_t *p_nal;
    int i_nal;
#if mcdull_SWATAW
	void* swataw;
#endif
}
McDull_t;

int McDull_open(McDull_t* McDull,
        int width, int height)
;
int McDull_close(McDull_t* McDull)
;
int McDull_get_qp(McDull_t* McDull)
;
int McDull_set_qp(McDull_t* McDull,
		int qp)
;
int McDull_set_keyint(McDull_t* McDull,
		int keyint)
;
int McDull_set_bframe(McDull_t* McDull,
		int bframe)
;
int McDull_encode_header(McDull_t* McDull,
        uint8_t g_264[], int32_t g_264_size[1])
;
int McDull_encode(McDull_t* McDull,
		uint8_t f_yuv[], 
		int i_frame_inp,
		uint8_t g_264[], int32_t g_264_size[1], 
		uint8_t g_yuv[])
;
int McDull_encode_flush(McDull_t* McDull,
		uint8_t g_264[], int32_t g_264_size[1], uint8_t g_yuv[])
;

#if mcdull_SWATAW
void McDull_swataw( McDull_t* McDull )
;

void McDull_swataw_I_FAST( McDull_t* McDull )
;
void McDull_swataw_I_GOOD( McDull_t* McDull )
;
void McDull_swataw_I_BEST( McDull_t* McDull )
;

void McDull_swataw_P_SKIP( McDull_t* McDull )
;
void McDull_swataw_P_FAST( McDull_t* McDull )
;
void McDull_swataw_P_GOOD( McDull_t* McDull )
;
void McDull_swataw_P_BEST( McDull_t* McDull )
;

void McDull_swataw_B_SKIP( McDull_t* McDull )
;
void McDull_swataw_B_FAST( McDull_t* McDull )
;
void McDull_swataw_B_GOOD( McDull_t* McDull )
;
void McDull_swataw_B_BEST( McDull_t* McDull )
;
#endif

int dull_encoder_encode( McDull_t *McDull,
        x264_nal_t **pp_nal, int *pi_nal,
        x264_picture_t *pic_in,
        x264_picture_t *pic_out )
;

#ifdef __cplusplus
}
#endif

#endif // _mcdull_encode_
