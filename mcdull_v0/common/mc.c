/*****************************************************************************
 * mc.c: motion compensation
 *****************************************************************************
 * Copyright (C) 2003-2010 x264 project
 *
 * Authors: Laurent Aimar <fenrir@via.ecp.fr>
 *          Loren Merritt <lorenm@u.washington.edu>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 * This program is also available under a commercial proprietary license.
 * For more information, contact us at licensing@x264.com.
 *****************************************************************************/

#include "common.h"

static inline void pixel_avg( pixel *dst,  int i_dst_stride,
                              pixel *src1, int i_src1_stride,
                              pixel *src2, int i_src2_stride,
                              int i_width, int i_height )
{
    int x, y;
    for( y = 0; y < i_height; y++ )
    {
        for( x = 0; x < i_width; x++ )
            dst[x] = ( src1[x] + src2[x] + 1 ) >> 1;
        dst  += i_dst_stride;
        src1 += i_src1_stride;
        src2 += i_src2_stride;
    }
}

static inline void pixel_avg_wxh( pixel *dst, int i_dst, pixel *src1, int i_src1, pixel *src2, int i_src2, int width, int height )
{
    int x, y;
    for( y = 0; y < height; y++ )
    {
        for( x = 0; x < width; x++ )
            dst[x] = ( src1[x] + src2[x] + 1 ) >> 1;
        src1 += i_src1;
        src2 += i_src2;
        dst += i_dst;
    }
}

/* Implicit weighted bipred only:
 * assumes log2_denom = 5, offset = 0, weight1 + weight2 = 64 */
#define op_scale2(x) dst[x] = x264_clip_pixel( (src1[x]*i_weight1 + src2[x]*i_weight2 + (1<<5)) >> 6 )
static inline void pixel_avg_weight_wxh( pixel *dst, int i_dst, pixel *src1, int i_src1, pixel *src2, int i_src2, int width, int height, int i_weight1 )
{
    int y;
    const int i_weight2 = 64 - i_weight1;
    for( y = 0; y<height; y++, dst += i_dst, src1 += i_src1, src2 += i_src2 )
    {
        op_scale2(0);
        op_scale2(1);
        if(width==2) continue;
        op_scale2(2);
        op_scale2(3);
        if(width==4) continue;
        op_scale2(4);
        op_scale2(5);
        op_scale2(6);
        op_scale2(7);
        if(width==8) continue;
        op_scale2(8);
        op_scale2(9);
        op_scale2(10);
        op_scale2(11);
        op_scale2(12);
        op_scale2(13);
        op_scale2(14);
        op_scale2(15);
    }
}
#undef op_scale2

#define PIXEL_AVG_C( name, width, height ) \
static void name( pixel *pix1, int i_stride_pix1, \
                  pixel *pix2, int i_stride_pix2, \
                  pixel *pix3, int i_stride_pix3, int weight ) \
{ \
    if( weight == 32 ) \
        pixel_avg_wxh( pix1, i_stride_pix1, pix2, i_stride_pix2, pix3, i_stride_pix3, width, height ); \
    else \
        pixel_avg_weight_wxh( pix1, i_stride_pix1, pix2, i_stride_pix2, pix3, i_stride_pix3, width, height, weight ); \
}
PIXEL_AVG_C( pixel_avg_16x16, 16, 16 )
PIXEL_AVG_C( pixel_avg_16x8,  16, 8 )
PIXEL_AVG_C( pixel_avg_8x16,  8, 16 )
PIXEL_AVG_C( pixel_avg_8x8,   8, 8 )
PIXEL_AVG_C( pixel_avg_8x4,   8, 4 )
PIXEL_AVG_C( pixel_avg_4x8,   4, 8 )
PIXEL_AVG_C( pixel_avg_4x4,   4, 4 )
PIXEL_AVG_C( pixel_avg_4x2,   4, 2 )
PIXEL_AVG_C( pixel_avg_2x4,   2, 4 )
PIXEL_AVG_C( pixel_avg_2x2,   2, 2 )

static void x264_weight_cache( x264_t *h, x264_weight_t *w )
{
    w->weightfn = h->mc.weight;
}
#define opscale(x) dst[x] = x264_clip_pixel( ((src[x] * scale + (1<<(denom - 1))) >> denom) + offset )
#define opscale_noden(x) dst[x] = x264_clip_pixel( src[x] * scale + offset )
static void mc_weight( pixel *dst, int i_dst_stride, pixel *src, int i_src_stride, const x264_weight_t *weight, int i_width, int i_height )
{
    int x, y;
    int offset = weight->i_offset << (BIT_DEPTH-8);
    int scale = weight->i_scale;
    int denom = weight->i_denom;
    if( denom >= 1 )
    {
        for( y = 0; y < i_height; y++, dst += i_dst_stride, src += i_src_stride )
            for( x = 0; x < i_width; x++ )
                opscale( x );
    }
    else
    {
        for( y = 0; y < i_height; y++, dst += i_dst_stride, src += i_src_stride )
            for( x = 0; x < i_width; x++ )
                opscale_noden( x );
    }
}

#define MC_WEIGHT_C( name, width ) \
    static void name( pixel *dst, int i_dst_stride, pixel *src, int i_src_stride, const x264_weight_t *weight, int height ) \
{ \
    mc_weight( dst, i_dst_stride, src, i_src_stride, weight, width, height );\
}

MC_WEIGHT_C( mc_weight_w20, 20 )
MC_WEIGHT_C( mc_weight_w16, 16 )
MC_WEIGHT_C( mc_weight_w12, 12 )
MC_WEIGHT_C( mc_weight_w8,   8 )
MC_WEIGHT_C( mc_weight_w4,   4 )
MC_WEIGHT_C( mc_weight_w2,   2 )

static weight_fn_t x264_mc_weight_wtab[6] =
{
    mc_weight_w2,
    mc_weight_w4,
    mc_weight_w8,
    mc_weight_w12,
    mc_weight_w16,
    mc_weight_w20,
};
const x264_weight_t weight_none[3] = { {{0}} };
static void mc_copy( pixel *src, int i_src_stride, pixel *dst, int i_dst_stride, int i_width, int i_height )
{
    int y;
    for( y = 0; y < i_height; y++ )
    {
        memcpy( dst, src, i_width * sizeof(pixel) );

        src += i_src_stride;
        dst += i_dst_stride;
    }
}

#define TAPFILTER(pix, d) ((pix)[x-2*d] + (pix)[x+3*d] - 5*((pix)[x-d] + (pix)[x+2*d]) + 20*((pix)[x] + (pix)[x+d]))
static void hpel_filter( pixel *dsth, pixel *dstv, pixel *dstc, pixel *src,
                         int stride, int width, int height, int16_t *buf )
{
    int x, y;
    const int pad = (BIT_DEPTH > 9) ? (-10 * PIXEL_MAX) : 0;
    for( y=0; y<height; y++ )
    {
        for( x = -2; x < width+3; x++ )
        {
            int v = TAPFILTER(src,stride);
            dstv[x] = x264_clip_pixel( (v + 16) >> 5 );
            /* transform v for storage in a 16-bit integer */
            buf[x+2] = v + pad;
        }
        for( x = 0; x < width; x++ )
            dstc[x] = x264_clip_pixel( (TAPFILTER(buf+2,1) - 32*pad + 512) >> 10 );
        for( x = 0; x < width; x++ )
            dsth[x] = x264_clip_pixel( (TAPFILTER(src,1) + 16) >> 5 );
        dsth += stride;
        dstv += stride;
        dstc += stride;
        src += stride;
    }
}

static const uint8_t hpel_ref0[16] = {0,1,1,1,0,1,1,1,2,3,3,3,0,1,1,1};
static const uint8_t hpel_ref1[16] = {0,0,0,0,2,2,3,2,2,2,3,2,2,2,3,2};

static void mc_luma( pixel *dst,    int i_dst_stride,
                     pixel *src[4], int i_src_stride,
                     int mvx, int mvy,
                     int i_width, int i_height, const x264_weight_t *weight )
{
    int qpel_idx = ((mvy&3)<<2) + (mvx&3);
    int offset = (mvy>>2)*i_src_stride + (mvx>>2);
    pixel *src1 = src[hpel_ref0[qpel_idx]] + offset + ((mvy&3) == 3) * i_src_stride;

    if( qpel_idx & 5 ) /* qpel interpolation needed */
    {
        pixel *src2 = src[hpel_ref1[qpel_idx]] + offset + ((mvx&3) == 3);
        pixel_avg( dst, i_dst_stride, src1, i_src_stride,
                   src2, i_src_stride, i_width, i_height );
        if( weight->weightfn )
            mc_weight( dst, i_dst_stride, dst, i_dst_stride, weight, i_width, i_height );
    }
    else if( weight->weightfn )
        mc_weight( dst, i_dst_stride, src1, i_src_stride, weight, i_width, i_height );
    else
        mc_copy( src1, i_src_stride, dst, i_dst_stride, i_width, i_height );
}

static pixel *get_ref( pixel *dst,   int *i_dst_stride,
                       pixel *src[4], int i_src_stride,
                       int mvx, int mvy,
                       int i_width, int i_height, const x264_weight_t *weight )
{
    int qpel_idx = ((mvy&3)<<2) + (mvx&3);
    int offset = (mvy>>2)*i_src_stride + (mvx>>2);
    pixel *src1 = src[hpel_ref0[qpel_idx]] + offset + ((mvy&3) == 3) * i_src_stride;

    if( qpel_idx & 5 ) /* qpel interpolation needed */
    {
        pixel *src2 = src[hpel_ref1[qpel_idx]] + offset + ((mvx&3) == 3);
        pixel_avg( dst, *i_dst_stride, src1, i_src_stride,
                   src2, i_src_stride, i_width, i_height );
        if( weight->weightfn )
            mc_weight( dst, *i_dst_stride, dst, *i_dst_stride, weight, i_width, i_height );
        return dst;
    }
    else if( weight->weightfn )
    {
        mc_weight( dst, *i_dst_stride, src1, i_src_stride, weight, i_width, i_height );
        return dst;
    }
    else
    {
        *i_dst_stride = i_src_stride;
        return src1;
    }
}

/* full chroma mc (ie until 1/8 pixel)*/
static void mc_chroma( pixel *dstu, pixel *dstv, int i_dst_stride,
                       pixel *src, int i_src_stride,
                       int mvx, int mvy,
                       int i_width, int i_height )
{
    pixel *srcp;
    int x, y;

    int d8x = mvx&0x07;
    int d8y = mvy&0x07;
    int cA = (8-d8x)*(8-d8y);
    int cB = d8x    *(8-d8y);
    int cC = (8-d8x)*d8y;
    int cD = d8x    *d8y;

    src += (mvy >> 3) * i_src_stride + (mvx >> 3)*2;
    srcp = &src[i_src_stride];

    for( y = 0; y < i_height; y++ )
    {
        for( x = 0; x < i_width; x++ )
        {
            dstu[x] = ( cA*src[2*x]  + cB*src[2*x+2] +
                        cC*srcp[2*x] + cD*srcp[2*x+2] + 32 ) >> 6;
            dstv[x] = ( cA*src[2*x+1]  + cB*src[2*x+3] +
                        cC*srcp[2*x+1] + cD*srcp[2*x+3] + 32 ) >> 6;
        }
        dstu += i_dst_stride;
        dstv += i_dst_stride;
        src   = srcp;
        srcp += i_src_stride;
    }
}

#define MC_COPY(W) \
static void mc_copy_w##W( pixel *dst, int i_dst, pixel *src, int i_src, int i_height ) \
{ \
    mc_copy( src, i_src, dst, i_dst, W, i_height ); \
}
MC_COPY( 16 )
MC_COPY( 8 )
MC_COPY( 4 )

void x264_plane_copy_c( pixel *dst, int i_dst,
                        uint8_t *src, int i_src, int w, int h)
{
    while( h-- )
    {
        memcpy( dst, src, w * sizeof(pixel) );
        dst += i_dst;
        src += i_src;
    }
}

void x264_plane_copy_interleave_c( pixel *dst, int i_dst,
                                   uint8_t *srcu, int i_srcu,
                                   uint8_t *srcv, int i_srcv, int w, int h )
{
    int x, y;
    for( y=0; y<h; y++, dst+=i_dst, srcu+=i_srcu, srcv+=i_srcv )
        for( x=0; x<w; x++ )
        {
            dst[2*x]   = ((pixel*)srcu)[x];
            dst[2*x+1] = ((pixel*)srcv)[x];
        }
}

void x264_plane_copy_deinterleave_c( pixel *dstu, int i_dstu,
                                     pixel *dstv, int i_dstv,
                                     pixel *src, int i_src, int w, int h )
{
    int x, y;
    for( y=0; y<h; y++, dstu+=i_dstu, dstv+=i_dstv, src+=i_src )
        for( x=0; x<w; x++ )
        {
            dstu[x] = src[2*x];
            dstv[x] = src[2*x+1];
        }
}

static void store_interleave_8x8x2( pixel *dst, int i_dst, pixel *srcu, pixel *srcv )
{
    int x, y;
    for( y=0; y<8; y++, dst+=i_dst, srcu+=FDEC_STRIDE, srcv+=FDEC_STRIDE )
        for( x=0; x<8; x++ )
        {
            dst[2*x]   = srcu[x];
            dst[2*x+1] = srcv[x];
        }
}

static void load_deinterleave_8x8x2_fenc( pixel *dst, pixel *src, int i_src )
{
    x264_plane_copy_deinterleave_c( dst, FENC_STRIDE, dst+FENC_STRIDE/2, FENC_STRIDE, src, i_src, 8, 8 );
}

static void load_deinterleave_8x8x2_fdec( pixel *dst, pixel *src, int i_src )
{
    x264_plane_copy_deinterleave_c( dst, FDEC_STRIDE, dst+FDEC_STRIDE/2, FDEC_STRIDE, src, i_src, 8, 8 );
}

static void prefetch_fenc_null( pixel *pix_y, int stride_y,
                                pixel *pix_uv, int stride_uv, int mb_x )
{}

static void prefetch_ref_null( pixel *pix, int stride, int parity )
{}

static void memzero_aligned( void * dst, int n )
{
    memset( dst, 0, n );
}

static void integral_init4h( uint16_t *sum, pixel *pix, int stride )
{
    int x, v = pix[0]+pix[1]+pix[2]+pix[3];
    for( x = 0; x < stride-4; x++ )
    {
        sum[x] = v + sum[x-stride];
        v += pix[x+4] - pix[x];
    }
}

static void integral_init8h( uint16_t *sum, pixel *pix, int stride )
{
    int x, v = pix[0]+pix[1]+pix[2]+pix[3]+pix[4]+pix[5]+pix[6]+pix[7];
    for( x = 0; x < stride-8; x++ )
    {
        sum[x] = v + sum[x-stride];
        v += pix[x+8] - pix[x];
    }
}

static void integral_init4v( uint16_t *sum8, uint16_t *sum4, int stride )
{
    int x;
    for( x = 0; x < stride-8; x++ )
        sum4[x] = sum8[x+4*stride] - sum8[x];
    for( x = 0; x < stride-8; x++ )
        sum8[x] = sum8[x+8*stride] + sum8[x+8*stride+4] - sum8[x] - sum8[x+4];
}

static void integral_init8v( uint16_t *sum8, int stride )
{
    int x;
    for( x = 0; x < stride-8; x++ )
        sum8[x] = sum8[x+8*stride] - sum8[x];
}

void x264_frame_init_lowres( x264_t *h, x264_frame_t *frame )
{
    pixel *src = frame->plane[0];
    int i_stride = frame->i_stride[0];
    int i_height = frame->i_lines[0];
    int i_width  = frame->i_width[0];
    int x, y;

    // duplicate last row and column so that their interpolation doesn't have to be special-cased
    for( y = 0; y < i_height; y++ )
        src[i_width+y*i_stride] = src[i_width-1+y*i_stride];
    memcpy( src+i_stride*i_height, src+i_stride*(i_height-1), (i_width+1) * sizeof(pixel) );
    h->mc.frame_init_lowres_core( src, frame->lowres[0], frame->lowres[1], frame->lowres[2], frame->lowres[3],
                                  i_stride, frame->i_stride_lowres, frame->i_width_lowres, frame->i_lines_lowres );
    x264_frame_expand_border_lowres( frame );

    memset( frame->i_cost_est, -1, sizeof(frame->i_cost_est) );

    for( y = 0; y < h->param.i_bframe + 2; y++ )
        for( x = 0; x < h->param.i_bframe + 2; x++ )
            frame->i_row_satds[y][x][0] = -1;

    for( y = 0; y <= !!h->param.i_bframe; y++ )
        for( x = 0; x <= h->param.i_bframe; x++ )
            frame->lowres_mvs[y][x][0][0] = 0x7FFF;
}

static void frame_init_lowres_core( pixel *src0, pixel *dst0, pixel *dsth, pixel *dstv, pixel *dstc,
                                    int src_stride, int dst_stride, int width, int height )
{
    int x, y;
    for( y = 0; y < height; y++ )
    {
        pixel *src1 = src0+src_stride;
        pixel *src2 = src1+src_stride;
        for( x = 0; x<width; x++ )
        {
            // slower than naive bilinear, but matches asm
#define FILTER(a,b,c,d) ((((a+b+1)>>1)+((c+d+1)>>1)+1)>>1)
            dst0[x] = FILTER(src0[2*x  ], src1[2*x  ], src0[2*x+1], src1[2*x+1]);
            dsth[x] = FILTER(src0[2*x+1], src1[2*x+1], src0[2*x+2], src1[2*x+2]);
            dstv[x] = FILTER(src1[2*x  ], src2[2*x  ], src1[2*x+1], src2[2*x+1]);
            dstc[x] = FILTER(src1[2*x+1], src2[2*x+1], src1[2*x+2], src2[2*x+2]);
#undef FILTER
        }
        src0 += src_stride*2;
        dst0 += dst_stride;
        dsth += dst_stride;
        dstv += dst_stride;
        dstc += dst_stride;
    }
}

#if defined(__GNUC__) && (ARCH_X86 || ARCH_X86_64)
// gcc isn't smart enough to use the "idiv" instruction
static ALWAYS_INLINE int32_t div_64_32(int64_t x, int32_t y)
{
    int32_t quotient, remainder;
    asm("idiv %4"
        :"=a"(quotient), "=d"(remainder)
        :"a"((uint32_t)x), "d"((int32_t)(x>>32)), "r"(y)
    );
    return quotient;
}
#else
#define div_64_32(x,y) ((x)/(y))
#endif

/* Estimate the total amount of influence on future quality that could be had if we
 * were to improve the reference samples used to inter predict any given macroblock. */
static void mbtree_propagate_cost( int *dst, uint16_t *propagate_in, uint16_t *intra_costs,
                                   uint16_t *inter_costs, uint16_t *inv_qscales, int len )
{
    int i;
    for( i = 0; i < len; i++ )
    {
        int propagate_amount = propagate_in[i] + ((intra_costs[i] * inv_qscales[i] + 128)>>8);
        dst[i] = div_64_32((int64_t)propagate_amount * (intra_costs[i] - (inter_costs[i] & LOWRES_COST_MASK)), intra_costs[i]);
    }
}

void x264_mc_init( int cpu, x264_mc_functions_t *pf )
{
    pf->mc_luma   = mc_luma;
    pf->get_ref   = get_ref;
    pf->mc_chroma = mc_chroma;

    pf->avg[PIXEL_16x16]= pixel_avg_16x16;
    pf->avg[PIXEL_16x8] = pixel_avg_16x8;
    pf->avg[PIXEL_8x16] = pixel_avg_8x16;
    pf->avg[PIXEL_8x8]  = pixel_avg_8x8;
    pf->avg[PIXEL_8x4]  = pixel_avg_8x4;
    pf->avg[PIXEL_4x8]  = pixel_avg_4x8;
    pf->avg[PIXEL_4x4]  = pixel_avg_4x4;
    pf->avg[PIXEL_4x2]  = pixel_avg_4x2;
    pf->avg[PIXEL_2x4]  = pixel_avg_2x4;
    pf->avg[PIXEL_2x2]  = pixel_avg_2x2;

    pf->weight    = x264_mc_weight_wtab;
    pf->offsetadd = x264_mc_weight_wtab;
    pf->offsetsub = x264_mc_weight_wtab;
    pf->weight_cache = x264_weight_cache;

    pf->copy_16x16_unaligned = mc_copy_w16;
    pf->copy[PIXEL_16x16] = mc_copy_w16;
    pf->copy[PIXEL_8x8]   = mc_copy_w8;
    pf->copy[PIXEL_4x4]   = mc_copy_w4;

    pf->store_interleave_8x8x2  = store_interleave_8x8x2;
    pf->load_deinterleave_8x8x2_fenc = load_deinterleave_8x8x2_fenc;
    pf->load_deinterleave_8x8x2_fdec = load_deinterleave_8x8x2_fdec;

    pf->plane_copy = x264_plane_copy_c;
    pf->plane_copy_interleave = x264_plane_copy_interleave_c;
    pf->plane_copy_deinterleave = x264_plane_copy_deinterleave_c;

    pf->hpel_filter = hpel_filter;

    pf->prefetch_fenc = prefetch_fenc_null;
    pf->prefetch_ref  = prefetch_ref_null;
    pf->memcpy_aligned = memcpy;
    pf->memzero_aligned = memzero_aligned;
    pf->frame_init_lowres_core = frame_init_lowres_core;

    pf->integral_init4h = integral_init4h;
    pf->integral_init8h = integral_init8h;
    pf->integral_init4v = integral_init4v;
    pf->integral_init8v = integral_init8v;

    pf->mbtree_propagate_cost = mbtree_propagate_cost;
}

void x264_frame_filter( x264_t *h, x264_frame_t *frame, int mb_y, int b_end )
{
    const int b_interlaced = h->sh.b_mbaff;
    const int stride = frame->i_stride[0] << b_interlaced;
    const int width = frame->i_width[0];
    int start = (mb_y*16 >> b_interlaced) - 8; // buffer = 4 for deblock + 3 for 6tap, rounded to 8
    int height = ((b_end ? frame->i_lines[0] : mb_y*16) >> b_interlaced) + 8;
    int offs = start*stride - 8; // buffer = 3 for 6tap, aligned to 8 for simd
    int y;

    if( mb_y & b_interlaced )
        return;

    for( y = 0; y <= b_interlaced; y++, offs += frame->i_stride[0] )
    {
        h->mc.hpel_filter(
            frame->filtered[1] + offs,
            frame->filtered[2] + offs,
            frame->filtered[3] + offs,
            frame->plane[0] + offs,
            stride, width + 16, height - start,
            h->scratch_buffer );
    }

    /* generate integral image:
     * frame->integral contains 2 planes. in the upper plane, each element is
     * the sum of an 8x8 pixel region with top-left corner on that point.
     * in the lower plane, 4x4 sums (needed only with --partitions p4x4). */

    if( frame->integral )
    {
        if( start < 0 )
        {
            memset( frame->integral - PADV * stride - PADH, 0, stride * sizeof(uint16_t) );
            start = -PADV;
        }
        if( b_end )
            height += PADV-9;
        for( y = start; y < height; y++ )
        {
            pixel    *pix  = frame->plane[0] + y * stride - PADH;
            uint16_t *sum8 = frame->integral + (y+1) * stride - PADH;
            uint16_t *sum4;
            if( h->frames.b_have_sub8x8_esa )
            {
                h->mc.integral_init4h( sum8, pix, stride );
                sum8 -= 8*stride;
                sum4 = sum8 + stride * (frame->i_lines[0] + PADV*2);
                if( y >= 8-PADV )
                    h->mc.integral_init4v( sum8, sum4, stride );
            }
            else
            {
                h->mc.integral_init8h( sum8, pix, stride );
                if( y >= 8-PADV )
                    h->mc.integral_init8v( sum8-8*stride, stride );
            }
        }
    }
}
