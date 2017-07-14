// mcdull.h

#ifndef _mcdull_
#define _mcdull_ 1

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define mcdull_SWATAW 1

#define mcdull_CODECTYPE_A 0
#define mcdull_CODECTYPE_F 1
#define mcdull_CODECTYPE_S 2

void* mcdull_new()
;
void mcdull_delete(void* p)
;

int mcdull_open(void* p,
		int width, int height)
;
int mcdull_close(void* p)
;
int mcdull_set_qp(void* p,
		int qp)
;
int mcdull_set_keyint(void* p,
		int keyint)
;
int mcdull_set_bframe( void* p,
		int bframe)
;
int mcdull_encode_header(void* p,
		int32_t g_264[], int32_t g_264_param[])
;
int mcdull_encode(void* p,
		int32_t f_yuv[], 
		int pts,
		int32_t g_264[], int32_t g_264_param[], 
		int32_t g_yuv[])
;
int mcdull_encode_flush(void* p,
		int32_t g_264[], int32_t g_264_param[], 
		int32_t g_yuv[])
;
#ifdef __cplusplus
}
#endif

#endif // _mcdull_
