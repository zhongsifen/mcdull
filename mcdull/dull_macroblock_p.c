/*****************************************************************************
 * macroblock.c: macroblock encoding
 *****************************************************************************
 * Copyright (C) 2003-2010 x264 project
 *
 * Authors: Laurent Aimar <fenrir@via.ecp.fr>
 *          Loren Merritt <lorenm@u.washington.edu>
 *          Jason Garrett-Glaser <darkshikari@gmail.com>
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
#ifdef _MCDULL_MACROBLOCK_P_

#include "mcdull_core.h"

/*****************************************************************************
 * x264_macroblock_encode_pskip:
 *  Encode an already marked skip block
 *****************************************************************************/
void dull_macroblock_encode_P_SKIP( x264_t *h )
{
    /* don't do pskip motion compensation if it was already done in macroblock_analyse */
    if( !h->mb.b_skip_mc )
    {
        int mvx = x264_clip3( h->mb.cache.mv[0][x264_scan8[0]][0],
                              h->mb.mv_min[0], h->mb.mv_max[0] );
        int mvy = x264_clip3( h->mb.cache.mv[0][x264_scan8[0]][1],
                              h->mb.mv_min[1], h->mb.mv_max[1] );

        h->mc.mc_luma( h->mb.pic.p_fdec[0],    FDEC_STRIDE,
                       h->mb.pic.p_fref[0][0], h->mb.pic.i_stride[0],
                       mvx, mvy, 16, 16, &h->sh.weight[0][0] );

        /* Special case for mv0, which is (of course) very common in P-skip mode. */
        if( mvx | mvy )
            h->mc.mc_chroma( h->mb.pic.p_fdec[1], h->mb.pic.p_fdec[2], FDEC_STRIDE,
                             h->mb.pic.p_fref[0][0][4], h->mb.pic.i_stride[1],
                             mvx, mvy, 8, 8 );
        else
            h->mc.load_deinterleave_8x8x2_fdec( h->mb.pic.p_fdec[1], h->mb.pic.p_fref[0][0][4], h->mb.pic.i_stride[1] );

        if( h->sh.weight[0][1].weightfn )
            h->sh.weight[0][1].weightfn[8>>2]( h->mb.pic.p_fdec[1], FDEC_STRIDE,
                                               h->mb.pic.p_fdec[1], FDEC_STRIDE,
                                               &h->sh.weight[0][1], 8 );
        if( h->sh.weight[0][2].weightfn )
            h->sh.weight[0][2].weightfn[8>>2]( h->mb.pic.p_fdec[2], FDEC_STRIDE,
                                               h->mb.pic.p_fdec[2], FDEC_STRIDE,
                                               &h->sh.weight[0][2], 8 );
    }

    x264_macroblock_encode_skip( h );
}
//{ macroblock_encode //{ macroblock_encode //{ macroblock_encode //{ macroblock_encode 
/*****************************************************************************
 * x264_macroblock_encode:
 *****************************************************************************/
void dull_macroblock_encode_P( x264_t *h )
{
    int cbp;
    int i_qp = h->mb.i_qp;
    int b_decimate = h->mb.b_dct_decimate;
    //int b_force_no_skip = 1;
    int i, idx, nz;
    h->mb.i_cbp_luma = 0;
    h->mb.cache.non_zero_count[x264_scan8[24]] = 0;

    if( h->mb.i_type == P_SKIP )
    {
        /* A bit special */
        x264_macroblock_encode_pskip( h );
        return;
    }
    if( h->mb.i_type == I_16x16 )
    {
        const int i_mode = h->mb.i_intra16x16_pred_mode;
        h->mb.b_transform_8x8 = 0;

        if( h->mb.b_lossless )
            x264_predict_lossless_16x16( h, i_mode );
        else
            h->predict_16x16[i_mode]( h->mb.pic.p_fdec[0] );

        /* encode the 16x16 macroblock */
        x264_mb_encode_i16x16( h, i_qp );
    }
    else if( h->mb.i_type == I_8x8 )
    {
        ALIGNED_ARRAY_16( pixel, edge,[33] );
        h->mb.b_transform_8x8 = 1;
        /* If we already encoded 3 of the 4 i8x8 blocks, we don't have to do them again. */
        if( h->mb.i_skip_intra )
        {
            h->mc.copy[PIXEL_16x16]( h->mb.pic.p_fdec[0], FDEC_STRIDE, h->mb.pic.i8x8_fdec_buf, 16, 16 );
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 0]] ) = h->mb.pic.i8x8_nnz_buf[0];
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 2]] ) = h->mb.pic.i8x8_nnz_buf[1];
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 8]] ) = h->mb.pic.i8x8_nnz_buf[2];
            M32( &h->mb.cache.non_zero_count[x264_scan8[10]] ) = h->mb.pic.i8x8_nnz_buf[3];
            h->mb.i_cbp_luma = h->mb.pic.i8x8_cbp;
            /* In RD mode, restore the now-overwritten DCT data. */
            if( h->mb.i_skip_intra == 2 )
                h->mc.memcpy_aligned( h->dct.luma8x8, h->mb.pic.i8x8_dct_buf, sizeof(h->mb.pic.i8x8_dct_buf) );
        }
        for( i = h->mb.i_skip_intra ? 3 : 0 ; i < 4; i++ )
        {
            pixel *p_dst = &h->mb.pic.p_fdec[0][8 * (i&1) + 8 * (i>>1) * FDEC_STRIDE];
            int i_mode = h->mb.cache.intra4x4_pred_mode[x264_scan8[4*i]];
            h->predict_8x8_filter( p_dst, edge, h->mb.i_neighbour8[i], x264_pred_i4x4_neighbors[i_mode] );

            if( h->mb.b_lossless )
                x264_predict_lossless_8x8( h, p_dst, i, i_mode, edge );
            else
                h->predict_8x8[i_mode]( p_dst, edge );

            x264_mb_encode_i8x8( h, i, i_qp );
        }
    }
    else if( h->mb.i_type == I_4x4 )
    {
        h->mb.b_transform_8x8 = 0;
        /* If we already encoded 15 of the 16 i4x4 blocks, we don't have to do them again. */
        if( h->mb.i_skip_intra )
        {
            h->mc.copy[PIXEL_16x16]( h->mb.pic.p_fdec[0], FDEC_STRIDE, h->mb.pic.i4x4_fdec_buf, 16, 16 );
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 0]] ) = h->mb.pic.i4x4_nnz_buf[0];
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 2]] ) = h->mb.pic.i4x4_nnz_buf[1];
            M32( &h->mb.cache.non_zero_count[x264_scan8[ 8]] ) = h->mb.pic.i4x4_nnz_buf[2];
            M32( &h->mb.cache.non_zero_count[x264_scan8[10]] ) = h->mb.pic.i4x4_nnz_buf[3];
            h->mb.i_cbp_luma = h->mb.pic.i4x4_cbp;
            /* In RD mode, restore the now-overwritten DCT data. */
            if( h->mb.i_skip_intra == 2 )
                h->mc.memcpy_aligned( h->dct.luma4x4, h->mb.pic.i4x4_dct_buf, sizeof(h->mb.pic.i4x4_dct_buf) );
        }
        for( i = h->mb.i_skip_intra ? 15 : 0 ; i < 16; i++ )
        {
            pixel *p_dst = &h->mb.pic.p_fdec[0][block_idx_xy_fdec[i]];
            int i_mode = h->mb.cache.intra4x4_pred_mode[x264_scan8[i]];

            if( (h->mb.i_neighbour4[i] & (MB_TOPRIGHT|MB_TOP)) == MB_TOP )
                /* emulate missing topright samples */
                MPIXEL_X4( &p_dst[4-FDEC_STRIDE] ) = PIXEL_SPLAT_X4( p_dst[3-FDEC_STRIDE] );

            if( h->mb.b_lossless )
                x264_predict_lossless_4x4( h, p_dst, i, i_mode );
            else
                h->predict_4x4[i_mode]( p_dst );
            x264_mb_encode_i4x4( h, i, i_qp );
        }
    }
    else    /* Inter MB */
//{ Inter MB //{ Inter MB //{ Inter MB //{ Inter MB 
    {
        int i8x8, i4x4;
        int i_decimate_mb = 0;

        /* Don't repeat motion compensation if it was already done in non-RD transform analysis */
        if( !h->mb.b_skip_mc )
            x264_mb_mc( h );

        if( h->mb.b_transform_8x8 )
        {
            ALIGNED_ARRAY_16( dctcoef, dct8x8,[4],[64] );
            b_decimate &= !h->mb.b_trellis || !h->param.b_cabac; // 8x8 trellis is inherently optimal decimation for CABAC
            h->dctf.sub16x16_dct8( dct8x8, h->mb.pic.p_fenc[0], h->mb.pic.p_fdec[0] );
            h->nr_count[1] += h->mb.b_noise_reduction * 4;

            for( idx = 0; idx < 4; idx++ )
            {
                if( h->mb.b_noise_reduction )
                    h->quantf.denoise_dct( dct8x8[idx], h->nr_residual_sum[1], h->nr_offset[1], 64 );
                nz = x264_quant_8x8( h, dct8x8[idx], i_qp, 0, idx );

                if( nz )
                {
                    h->zigzagf.scan_8x8( h->dct.luma8x8[idx], dct8x8[idx] );
                    if( b_decimate )
                    {
                        int i_decimate_8x8 = h->quantf.decimate_score64( h->dct.luma8x8[idx] );
                        i_decimate_mb += i_decimate_8x8;
                        if( i_decimate_8x8 >= 4 )
                            h->mb.i_cbp_luma |= 1<<idx;
                    }
                    else
                        h->mb.i_cbp_luma |= 1<<idx;
                }
            }

            if( i_decimate_mb < 6 && b_decimate )
            {
                h->mb.i_cbp_luma = 0;
                CLEAR_16x16_NNZ
            }
            else
            {
                for( idx = 0; idx < 4; idx++ )
                {
                    int x = idx&1;
                    int y = idx>>1;
                    int s8 = X264_SCAN8_0 + 2*x + 16*y;

                    if( h->mb.i_cbp_luma&(1<<idx) )
                    {
                        h->quantf.dequant_8x8( dct8x8[idx], h->dequant8_mf[CQM_8PY], i_qp );
                        h->dctf.add8x8_idct8( &h->mb.pic.p_fdec[0][8*x + 8*y*FDEC_STRIDE], dct8x8[idx] );
                        STORE_8x8_NNZ( s8, 1 );
                    }
                    else
                        STORE_8x8_NNZ( s8, 0 );
                }
            }
        }
        else
        {
            ALIGNED_ARRAY_16( dctcoef, dct4x4,[16],[16] );
            h->dctf.sub16x16_dct( dct4x4, h->mb.pic.p_fenc[0], h->mb.pic.p_fdec[0] );
            h->nr_count[0] += h->mb.b_noise_reduction * 16;

            for( i8x8 = 0; i8x8 < 4; i8x8++ )
            {
                int i_decimate_8x8 = 0;
                int cbp = 0;
                int x, y;

                /* encode one 4x4 block */
                for( i4x4 = 0; i4x4 < 4; i4x4++ )
                {
                    idx = i8x8 * 4 + i4x4;

                    if( h->mb.b_noise_reduction )
                        h->quantf.denoise_dct( dct4x4[idx], h->nr_residual_sum[0], h->nr_offset[0], 16 );
                    nz = x264_quant_4x4( h, dct4x4[idx], i_qp, DCT_LUMA_4x4, 0, idx );
                    h->mb.cache.non_zero_count[x264_scan8[idx]] = nz;

                    if( nz )
                    {
                        h->zigzagf.scan_4x4( h->dct.luma4x4[idx], dct4x4[idx] );
                        h->quantf.dequant_4x4( dct4x4[idx], h->dequant4_mf[CQM_4PY], i_qp );
                        if( b_decimate && i_decimate_8x8 < 6 )
                            i_decimate_8x8 += h->quantf.decimate_score16( h->dct.luma4x4[idx] );
                        cbp = 1;
                    }
                }

                x = i8x8&1;
                y = i8x8>>1;

                /* decimate this 8x8 block */
                i_decimate_mb += i_decimate_8x8;
                if( b_decimate )
                {
                    if( i_decimate_8x8 < 4 )
                    {
                        int s8 = X264_SCAN8_0 + 2*x + 16*y;
                        STORE_8x8_NNZ( s8, 0 );
                    }
                    else
                        h->mb.i_cbp_luma |= 1<<i8x8;
                }
                else if( cbp )
                {
                    h->dctf.add8x8_idct( &h->mb.pic.p_fdec[0][8*x + 8*y*FDEC_STRIDE], &dct4x4[i8x8*4] );
                    h->mb.i_cbp_luma |= 1<<i8x8;
                }
            }

            if( b_decimate )
            {
                if( i_decimate_mb < 6 )
                {
                    h->mb.i_cbp_luma = 0;
                    CLEAR_16x16_NNZ
                }
                else
                {
                    for( i8x8 = 0; i8x8 < 4; i8x8++ )
                        if( h->mb.i_cbp_luma&(1<<i8x8) )
                            h->dctf.add8x8_idct( &h->mb.pic.p_fdec[0][(i8x8&1)*8 + (i8x8>>1)*8*FDEC_STRIDE], &dct4x4[i8x8*4] );
                }
            }
        }
    }

    /* encode chroma */
    if( IS_INTRA( h->mb.i_type ) )
    {
        const int i_mode = h->mb.i_chroma_pred_mode;
        if( h->mb.b_lossless )
            x264_predict_lossless_8x8_chroma( h, i_mode );
        else
        {
            h->predict_8x8c[i_mode]( h->mb.pic.p_fdec[1] );
            h->predict_8x8c[i_mode]( h->mb.pic.p_fdec[2] );
        }
    }

    /* encode the 8x8 blocks */
    x264_mb_encode_8x8_chroma( h, !IS_INTRA( h->mb.i_type ), h->mb.i_chroma_qp );

    /* store cbp */
    cbp = h->mb.i_cbp_chroma << 4 | h->mb.i_cbp_luma;
    if( h->param.b_cabac )
        cbp |= h->mb.cache.non_zero_count[x264_scan8[24]] << 8
            |  h->mb.cache.non_zero_count[x264_scan8[25]] << 9
            |  h->mb.cache.non_zero_count[x264_scan8[26]] << 10;
    h->mb.cbp[h->mb.i_mb_xy] = cbp;

    /* Check for P_SKIP
     * XXX: in the me perhaps we should take x264_mb_predict_mv_pskip into account
     *      (if multiple mv give same result)*/
    //if( !b_force_no_skip )
    //{
        if( h->mb.i_type == P_L0 && h->mb.i_partition == D_16x16 &&
            !(h->mb.i_cbp_luma | h->mb.i_cbp_chroma) &&
            M32( h->mb.cache.mv[0][x264_scan8[0]] ) == M32( h->mb.cache.pskip_mv )
            && h->mb.cache.ref[0][x264_scan8[0]] == 0 )
        {
            h->mb.i_type = P_SKIP;
        }
    //}
}
//} macroblock_encode //} macroblock_encode //} macroblock_encode //} macroblock_encode 

#endif // _MCDULL_MACROBLOCK_P_
