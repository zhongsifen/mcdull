// mcdull_x264.h

#ifndef _mcdull_x264_
#define _mcdull_x264_ 1

#include "common/common.h"
#include "encoder/ratecontrol.h"

#ifdef __cplusplus
extern "C" {
#endif

void x264_plane_copy_c( 
		pixel *dst, int i_dst,
		pixel *src, int i_src, 
		int w, int h )
;
void x264_plane_copy_interleave_c( 
		pixel *dst,  int i_dst,
		pixel *srcu, int i_srcu,
		pixel *srcv, int i_srcv, 
		int w, int h )
;
void x264_plane_copy_deinterleave_c( 
		pixel *dstu, int i_dstu,
		pixel *dstv, int i_dstv,
		pixel *src,  int i_src, 
		int w, int h )
;
#ifdef __cplusplus
}
#endif

#endif // _mcdull_x264_
