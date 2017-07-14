// mcdull_dull.h

#ifndef _mcdull_dull_
#define _mcdull_dull_ 1

#include "common/common.h"
#include "encoder/macroblock.h"
#include "encoder/me.h"

#ifdef __cplusplus
extern "C" {
#endif

int dull_encoder_headers( x264_t *h, x264_nal_t **pp_nal, int *pi_nal )
;
void dull_macroblock_analyse( x264_t *h )
;
void dull_macroblock_analyse_P( x264_t *h )
;
void dull_macroblock_analyse_P_SKIP( x264_t *h )
;
void dull_macroblock_analyse_P_FAST( x264_t *h )
;
void dull_macroblock_analyse_P_GOOD( x264_t *h )
;
void dull_macroblock_analyse_P_BEST( x264_t *h )
;

void dull_macroblock_analyse_B( x264_t *h )
;
void dull_macroblock_analyse_B_SKIP( x264_t *h )
;
void dull_macroblock_analyse_B_FAST( x264_t *h )
;
void dull_macroblock_analyse_B_GOOD( x264_t *h )
;
void dull_macroblock_analyse_B_BEST( x264_t *h )
;

void dull_macroblock_encode_P( x264_t *h )
;
void dull_macroblock_encode_P_SKIP( x264_t *h )
;
void dull_macroblock_encode_P_FAST( x264_t *h )
;

void dull_macroblock_encode_B( x264_t *h )
;
void dull_macroblock_encode_B_SKIP( x264_t *h )
;

void dull_me_search_ref( x264_t *h, x264_me_t *m, int16_t (*mvc)[2], int i_mvc, int *p_halfpel_thresh )
;
void dull_me_search_ref_0( x264_t *h, x264_me_t *m, int16_t (*mvc)[2], int i_mvc, int *p_halfpel_thresh )
;
void dull_me_search_ref_1( x264_t *h, x264_me_t *m, int16_t (*mvc)[2], int i_mvc, int *p_halfpel_thresh )
;
void dull_mb_mc( x264_t *h )
;

#ifdef __cplusplus
}
#endif

#endif // _mcdull_dull_
