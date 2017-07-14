/*****************************************************************************
 * analyse.c: macroblock analysis
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

#ifdef _MCDULL_ANALYSE_

#include "mcdull_core.h"

static void dull_analyse_update_cache( x264_t *h, x264_mb_analysis_t *a );

//{ mb_analyse_int //{ mb_analyse_int //{ mb_analyse_int //{ mb_analyse_int 
static void dull_mb_analyse_init( x264_t *h, x264_mb_analysis_t *a )
{
    int subme = h->param.analyse.i_subpel_refine - (h->sh.i_type == SLICE_TYPE_B);

    /* mbrd == 1 -> RD mode decision */
    /* mbrd == 2 -> RD refinement */
    /* mbrd == 3 -> QPRD */
    a->i_mbrd = (subme>=6) + (subme>=8) + (h->param.analyse.i_subpel_refine>=10);
    h->mb.b_deblock_rdo = h->param.analyse.i_subpel_refine >= 9 && h->sh.i_disable_deblocking_filter_idc != 1;

    x264_mb_analyse_init_qp( h, a, h->mb.i_qp );

    h->mb.b_transform_8x8 = 0;
    h->mb.b_noise_reduction = 0;

    /* I: Intra part */
    a->i_satd_i16x16 =
    a->i_satd_i8x8   =
    a->i_satd_i4x4   =
    a->i_satd_i8x8chroma = COST_MAX;

    /* non-RD PCM decision is inaccurate (as is psy-rd), so don't do it */
    a->i_satd_pcm = !h->mb.i_psy_rd && a->i_mbrd ? ((uint64_t)X264_PCM_COST*a->i_lambda2 + 128) >> 8 : COST_MAX;

    a->b_fast_intra = 0;
    a->b_avoid_topright = 0;
    h->mb.i_skip_intra =
        h->mb.b_lossless ? 0 :
        a->i_mbrd ? 2 :
        !h->param.analyse.i_trellis && !h->param.analyse.i_noise_reduction;

    /* II: Inter part P/B frame */
    if( h->sh.i_type != SLICE_TYPE_I )
    {
        int i, j;
        int i_fmv_range = 4 * h->param.analyse.i_mv_range;
        // limit motion search to a slightly smaller range than the theoretical limit,
        // since the search may go a few iterations past its given range
        int i_fpel_border = 6; // umh: 1 for diamond, 2 for octagon, 2 for hpel

        /* Calculate max allowed MV range */
#define CLIP_FMV(mv) x264_clip3( mv, -i_fmv_range, i_fmv_range-1 )
        h->mb.mv_min[0] = 4*( -16*h->mb.i_mb_x - 24 );
        h->mb.mv_max[0] = 4*( 16*( h->sps->i_mb_width - h->mb.i_mb_x - 1 ) + 24 );
        h->mb.mv_min_spel[0] = CLIP_FMV( h->mb.mv_min[0] );
        h->mb.mv_max_spel[0] = CLIP_FMV( h->mb.mv_max[0] );
        if( h->param.b_intra_refresh && h->sh.i_type == SLICE_TYPE_P )
        {
            int max_x = (h->fref0[0]->i_pir_end_col * 16 - 3)*4; /* 3 pixels of hpel border */
            int max_mv = max_x - 4*16*h->mb.i_mb_x;
            /* If we're left of the refresh bar, don't reference right of it. */
            if( max_mv > 0 && h->mb.i_mb_x < h->fdec->i_pir_start_col )
                h->mb.mv_max_spel[0] = X264_MIN( h->mb.mv_max_spel[0], max_mv );
        }
        h->mb.mv_min_fpel[0] = (h->mb.mv_min_spel[0]>>2) + i_fpel_border;
        h->mb.mv_max_fpel[0] = (h->mb.mv_max_spel[0]>>2) - i_fpel_border;
        if( h->mb.i_mb_x == 0 )
        {
            int mb_y = h->mb.i_mb_y;
            int mb_height = h->sps->i_mb_height;
            int thread_mvy_range = i_fmv_range;

            if( h->i_thread_frames > 1 )
            {
                int pix_y = (h->mb.i_mb_y | h->mb.b_interlaced) * 16;
                int thresh = pix_y + h->param.analyse.i_mv_range_thread;
                for( i = (h->sh.i_type == SLICE_TYPE_B); i >= 0; i-- )
                {
                    x264_frame_t **fref = i ? h->fref1 : h->fref0;
                    int i_ref = i ? h->i_ref1 : h->i_ref0;
                    for( j = 0; j < i_ref; j++ )
                    {
                        x264_frame_cond_wait( fref[j]->orig, thresh );
                        thread_mvy_range = X264_MIN( thread_mvy_range, fref[j]->orig->i_lines_completed - pix_y );
                    }
                }

                if( h->param.b_deterministic )
                    thread_mvy_range = h->param.analyse.i_mv_range_thread;
                if( h->mb.b_interlaced )
                    thread_mvy_range >>= 1;

                x264_analyse_weight_frame( h, pix_y + thread_mvy_range );
            }

            h->mb.mv_min[1] = 4*( -16*mb_y - 24 );
            h->mb.mv_max[1] = 4*( 16*( mb_height - mb_y - 1 ) + 24 );
            h->mb.mv_min_spel[1] = x264_clip3( h->mb.mv_min[1], -i_fmv_range, i_fmv_range );
            h->mb.mv_max_spel[1] = CLIP_FMV( h->mb.mv_max[1] );
            h->mb.mv_max_spel[1] = X264_MIN( h->mb.mv_max_spel[1], thread_mvy_range*4 );
            h->mb.mv_min_fpel[1] = (h->mb.mv_min_spel[1]>>2) + i_fpel_border;
            h->mb.mv_max_fpel[1] = (h->mb.mv_max_spel[1]>>2) - i_fpel_border;
        }
#undef CLIP_FMV

        a->l0.me16x16.cost =
        a->l0.i_rd16x16    =
        a->l0.i_cost8x8    =
        a->l0.i_cost16x8   =
        a->l0.i_cost8x16   = COST_MAX;
        if( h->sh.i_type == SLICE_TYPE_B )
        {
            a->l1.me16x16.cost =
            a->l1.i_rd16x16    =
            a->l1.i_cost8x8    =
            a->i_cost8x8direct[0] =
            a->i_cost8x8direct[1] =
            a->i_cost8x8direct[2] =
            a->i_cost8x8direct[3] =
            a->l1.i_cost16x8   =
            a->l1.i_cost8x16   =
            a->i_rd16x16bi     =
            a->i_rd16x16direct =
            a->i_rd8x8bi       =
            a->i_rd16x8bi      =
            a->i_rd8x16bi      =
            a->i_cost16x16bi   =
            a->i_cost16x16direct =
            a->i_cost8x8bi     =
            a->i_cost16x8bi    =
            a->i_cost8x16bi    = COST_MAX;
        }
        else if( h->param.analyse.inter & X264_ANALYSE_PSUB8x8 )
            for( i = 0; i < 4; i++ )
            {
                a->l0.i_cost4x4[i] =
                a->l0.i_cost8x4[i] =
                a->l0.i_cost4x8[i] = COST_MAX;
            }

        /* Fast intra decision */
        if( h->mb.i_mb_xy - h->sh.i_first_mb > 4 )
        {
            /* Always run in fast-intra mode for subme < 3 */
            if( h->mb.i_subpel_refine > 2 &&
              ( IS_INTRA( h->mb.i_mb_type_left ) ||
                IS_INTRA( h->mb.i_mb_type_top ) ||
                IS_INTRA( h->mb.i_mb_type_topleft ) ||
                IS_INTRA( h->mb.i_mb_type_topright ) ||
                (h->sh.i_type == SLICE_TYPE_P && IS_INTRA( h->fref0[0]->mb_type[h->mb.i_mb_xy] )) ||
                (h->mb.i_mb_xy - h->sh.i_first_mb < 3*(h->stat.frame.i_mb_count[I_4x4] + h->stat.frame.i_mb_count[I_8x8] + h->stat.frame.i_mb_count[I_16x16])) ) )
            { /* intra is likely */ }
            else
            {
                a->b_fast_intra = 1;
            }
        }
        h->mb.b_skip_mc = 0;
        if( h->param.b_intra_refresh && h->sh.i_type == SLICE_TYPE_P &&
            h->mb.i_mb_x >= h->fdec->i_pir_start_col && h->mb.i_mb_x <= h->fdec->i_pir_end_col )
        {
            a->b_force_intra = 1;
            a->b_fast_intra = 0;
            a->b_avoid_topright = h->mb.i_mb_x == h->fdec->i_pir_end_col;
        }
        else
            a->b_force_intra = 0;
    }
}
//} mb_analyse_int //} mb_analyse_int //} mb_analyse_int //} mb_analyse_int 

//{ mb_analyse_inter_p16x16 //{ mb_analyse_inter_p16x16 //{ mb_analyse_inter_p16x16 //{ mb_analyse_inter_p16x16
static void dull_mb_analyse_inter_p16x16( x264_t *h, x264_mb_analysis_t *a )
{
    x264_me_t m;
    int i_ref, i_mvc;
    ALIGNED_4( int16_t mvc[8][2] );
    int i_halfpel_thresh = INT_MAX;
    int *p_halfpel_thresh = h->mb.pic.i_fref[0]>1 ? &i_halfpel_thresh : NULL;

    /* 16x16 Search on all ref frame */
    m.i_pixel = PIXEL_16x16;
    LOAD_FENC( &m, h->mb.pic.p_fenc, 0, 0 );

    a->l0.me16x16.cost = INT_MAX;
    for( i_ref = 0; i_ref < h->mb.pic.i_fref[0]; i_ref++ )
    {
        m.i_ref_cost = REF_COST( 0, i_ref );
        i_halfpel_thresh -= m.i_ref_cost;

        /* search with ref */
        LOAD_HPELS( &m, h->mb.pic.p_fref[0][i_ref], 0, i_ref, 0, 0 );
        LOAD_WPELS( &m, h->mb.pic.p_fref_w[i_ref], 0, i_ref, 0, 0 );

        x264_mb_predict_mv_16x16( h, 0, i_ref, m.mvp );

        if( h->mb.ref_blind_dupe == i_ref )
        {
            CP32( m.mv, a->l0.mvc[0][0] );
            x264_me_refine_qpel_refdupe( h, &m, p_halfpel_thresh );
        }
        else
        {
            x264_mb_predict_mv_ref16x16( h, 0, i_ref, mvc, &i_mvc );
            x264_me_search_ref( h, &m, mvc, i_mvc, p_halfpel_thresh );
        }

        /* save mv for predicting neighbors */
        CP32( h->mb.mvr[0][i_ref][h->mb.i_mb_xy], m.mv );
        CP32( a->l0.mvc[i_ref][0], m.mv );

        /* early termination
         * SSD threshold would probably be better than SATD */
        if( i_ref == 0
            && a->b_try_skip
            && m.cost-m.cost_mv < 300*a->i_lambda
            &&  abs(m.mv[0]-h->mb.cache.pskip_mv[0])
              + abs(m.mv[1]-h->mb.cache.pskip_mv[1]) <= 1
            && x264_macroblock_probe_pskip( h ) )
        {
            h->mb.i_type = P_SKIP;
            x264_analyse_update_cache( h, a );
            assert( h->mb.cache.pskip_mv[1] <= h->mb.mv_max_spel[1] || h->i_thread_frames == 1 );
            return;
        }

        m.cost += m.i_ref_cost;
        i_halfpel_thresh += m.i_ref_cost;

        if( m.cost < a->l0.me16x16.cost )
            h->mc.memcpy_aligned( &a->l0.me16x16, &m, sizeof(x264_me_t) );
    }

    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, a->l0.me16x16.i_ref );
    assert( a->l0.me16x16.mv[1] <= h->mb.mv_max_spel[1] || h->i_thread_frames == 1 );

    h->mb.i_type = P_L0;
    if( a->i_mbrd )
    {
        x264_mb_init_fenc_cache( h, a->i_mbrd >= 2 || h->param.analyse.inter & X264_ANALYSE_PSUB8x8 );
        if( a->l0.me16x16.i_ref == 0 && M32( a->l0.me16x16.mv ) == M32( h->mb.cache.pskip_mv ) && !a->b_force_intra )
        {
            h->mb.i_partition = D_16x16;
            x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 0, a->l0.me16x16.mv );
            a->l0.i_rd16x16 = x264_rd_cost_mb( h, a->i_lambda2 );
            if( !(h->mb.i_cbp_luma|h->mb.i_cbp_chroma) )
                h->mb.i_type = P_SKIP;
        }
    }
}
//} mb_analyse_inter_p16x16 //} mb_analyse_inter_p16x16 //} mb_analyse_inter_p16x16 //} mb_analyse_inter_p16x16

//{ mb_analyse_inter_p8x8 //{ mb_analyse_inter_p8x8 //{ mb_analyse_inter_p8x8 //{ mb_analyse_inter_p8x8
static void dull_mb_analyse_inter_p8x8( x264_t *h, x264_mb_analysis_t *a )
{
    /* Duplicate refs are rarely useful in p8x8 due to the high cost of the
     * reference frame flags.  Thus, if we're not doing mixedrefs, just
     * don't bother analysing the dupes. */
    const int i_ref = h->mb.ref_blind_dupe == a->l0.me16x16.i_ref ? 0 : a->l0.me16x16.i_ref;
    const int i_ref_cost = h->param.b_cabac || i_ref ? REF_COST( 0, i_ref ) : 0;
    pixel **p_fenc = h->mb.pic.p_fenc;
    int i_mvc;
    int16_t (*mvc)[2] = a->l0.mvc[i_ref];
    int i;

    /* XXX Needed for x264_mb_predict_mv */
    h->mb.i_partition = D_8x8;

    i_mvc = 1;
    CP32( mvc[0], a->l0.me16x16.mv );

    for( i = 0; i < 4; i++ )
    {
        x264_me_t *m = &a->l0.me8x8[i];
        int x8 = i&1;
        int y8 = i>>1;

        m->i_pixel = PIXEL_8x8;
        m->i_ref_cost = i_ref_cost;

        LOAD_FENC( m, p_fenc, 8*x8, 8*y8 );
        LOAD_HPELS( m, h->mb.pic.p_fref[0][i_ref], 0, i_ref, 8*x8, 8*y8 );
        LOAD_WPELS( m, h->mb.pic.p_fref_w[i_ref], 0, i_ref, 8*x8, 8*y8 );

        x264_mb_predict_mv( h, 0, 4*i, 2, m->mvp );
        x264_me_search( h, m, mvc, i_mvc );

        x264_macroblock_cache_mv_ptr( h, 2*x8, 2*y8, 2, 2, 0, m->mv );

        CP32( mvc[i_mvc], m->mv );
        i_mvc++;

        a->i_satd8x8[0][i] = m->cost - m->cost_mv;

        /* mb type cost */
        m->cost += i_ref_cost;
        if( !h->param.b_cabac || (h->param.analyse.inter & X264_ANALYSE_PSUB8x8) )
            m->cost += a->i_lambda * i_sub_mb_p_cost_table[D_L0_8x8];
    }

    a->l0.i_cost8x8 = a->l0.me8x8[0].cost + a->l0.me8x8[1].cost +
                      a->l0.me8x8[2].cost + a->l0.me8x8[3].cost;
    /* theoretically this should include 4*ref_cost,
     * but 3 seems a better approximation of cabac. */
    if( h->param.b_cabac )
        a->l0.i_cost8x8 -= i_ref_cost;
    h->mb.i_sub_partition[0] = h->mb.i_sub_partition[1] =
    h->mb.i_sub_partition[2] = h->mb.i_sub_partition[3] = D_L0_8x8;
}
//} mb_analyse_inter_p8x8 //} mb_analyse_inter_p8x8 //} mb_analyse_inter_p8x8 //} mb_analyse_inter_p8x8

//{ mb_analyse_inter_b16x16 //{ mb_analyse_inter_b16x16 //{ mb_analyse_inter_b16x16 //{ mb_analyse_inter_b16x16
static void dull_mb_analyse_inter_b16x16( x264_t *h, x264_mb_analysis_t *a )
{
    ALIGNED_ARRAY_16( pixel, pix0,[16*16] );
    ALIGNED_ARRAY_16( pixel, pix1,[16*16] );
    pixel *src0, *src1;
    int stride0 = 16, stride1 = 16, ref_costs;
    int i_ref, i_mvc, l;
    ALIGNED_4( int16_t mvc[9][2] );
    int try_skip = a->b_try_skip;
    int list1_skipped = 0;
    int i_halfpel_thresh[2] = {INT_MAX, INT_MAX};
    int *p_halfpel_thresh[2] = {h->mb.pic.i_fref[0]>1 ? &i_halfpel_thresh[0] : NULL,
                                h->mb.pic.i_fref[1]>1 ? &i_halfpel_thresh[1] : NULL};

    x264_me_t m;
    m.i_pixel = PIXEL_16x16;

    LOAD_FENC( &m, h->mb.pic.p_fenc, 0, 0 );

    /* 16x16 Search on list 0 and list 1 */
    a->l0.me16x16.cost = INT_MAX;
    a->l1.me16x16.cost = INT_MAX;
    for( l = 1; l >= 0; )
    {
        x264_mb_analysis_list_t *lX = l ? &a->l1 : &a->l0;

        /* This loop is extremely munged in order to facilitate the following order of operations,
         * necessary for an efficient fast skip.
         * 1.  Search list1 ref0.
         * 2.  Search list0 ref0.
         * 3.  Try skip.
         * 4.  Search the rest of list0.
         * 5.  Go back and finish list1.
         */
        for( i_ref = (list1_skipped && l == 1) ? 1 : 0; i_ref < h->mb.pic.i_fref[l]; i_ref++ )
        {
            if( try_skip && l == 1 && i_ref > 0 )
            {
                list1_skipped = 1;
                break;
            }

            m.i_ref_cost = REF_COST( l, i_ref );

            /* search with ref */
            LOAD_HPELS( &m, h->mb.pic.p_fref[l][i_ref], l, i_ref, 0, 0 );
            x264_mb_predict_mv_16x16( h, l, i_ref, m.mvp );
            x264_mb_predict_mv_ref16x16( h, l, i_ref, mvc, &i_mvc );
            x264_me_search_ref( h, &m, mvc, i_mvc, p_halfpel_thresh[l] );

            /* add ref cost */
            m.cost += m.i_ref_cost;

            if( m.cost < lX->me16x16.cost )
                h->mc.memcpy_aligned( &lX->me16x16, &m, sizeof(x264_me_t) );

            /* save mv for predicting neighbors */
            CP32( lX->mvc[i_ref][0], m.mv );
            CP32( h->mb.mvr[l][i_ref][h->mb.i_mb_xy], m.mv );

            /* Fast skip detection. */
            if( i_ref == 0 && try_skip )
            {
                if( abs(lX->me16x16.mv[0]-h->mb.cache.direct_mv[l][0][0]) +
                    abs(lX->me16x16.mv[1]-h->mb.cache.direct_mv[l][0][1]) > 1 )
                {
                    try_skip = 0;
                }
                else if( !l )
                {
                    /* We already tested skip */
                    h->mb.i_type = B_SKIP;
                    x264_analyse_update_cache( h, a );
                    return;
                }
            }
        }
        if( list1_skipped && l == 1 && i_ref == h->mb.pic.i_fref[1] )
            break;
        if( list1_skipped && l == 0 )
            l = 1;
        else
            l--;
    }

    /* get cost of BI mode */
    h->mc.memcpy_aligned( &a->l0.bi16x16, &a->l0.me16x16, sizeof(x264_me_t) );
    h->mc.memcpy_aligned( &a->l1.bi16x16, &a->l1.me16x16, sizeof(x264_me_t) );
    ref_costs = REF_COST( 0, a->l0.bi16x16.i_ref ) + REF_COST( 1, a->l1.bi16x16.i_ref );
    src0 = h->mc.get_ref( pix0, &stride0,
                          h->mb.pic.p_fref[0][a->l0.bi16x16.i_ref], h->mb.pic.i_stride[0],
                          a->l0.bi16x16.mv[0], a->l0.bi16x16.mv[1], 16, 16, weight_none );
    src1 = h->mc.get_ref( pix1, &stride1,
                          h->mb.pic.p_fref[1][a->l1.bi16x16.i_ref], h->mb.pic.i_stride[0],
                          a->l1.bi16x16.mv[0], a->l1.bi16x16.mv[1], 16, 16, weight_none );

    h->mc.avg[PIXEL_16x16]( pix0, 16, src0, stride0, src1, stride1, h->mb.bipred_weight[a->l0.bi16x16.i_ref][a->l1.bi16x16.i_ref] );

    a->i_cost16x16bi = h->pixf.mbcmp[PIXEL_16x16]( h->mb.pic.p_fenc[0], FENC_STRIDE, pix0, 16 )
                     + ref_costs
                     + a->l0.bi16x16.cost_mv
                     + a->l1.bi16x16.cost_mv;

    if( h->mb.b_chroma_me )
        a->i_cost16x16bi += x264_analyse_bi_chroma( h, a, 0, PIXEL_16x16 );

    /* Always try the 0,0,0,0 vector; helps avoid errant motion vectors in fades */
    if( M32( a->l0.bi16x16.mv ) | M32( a->l1.bi16x16.mv ) )
    {
        int cost00;
        int l0_mv_cost = a->l0.bi16x16.p_cost_mv[-a->l0.bi16x16.mvp[0]]
                       + a->l0.bi16x16.p_cost_mv[-a->l0.bi16x16.mvp[1]];
        int l1_mv_cost = a->l1.bi16x16.p_cost_mv[-a->l1.bi16x16.mvp[0]]
                       + a->l1.bi16x16.p_cost_mv[-a->l1.bi16x16.mvp[1]];
        h->mc.avg[PIXEL_16x16]( pix0, 16, h->mb.pic.p_fref[0][a->l0.bi16x16.i_ref][0], h->mb.pic.i_stride[0],
                                h->mb.pic.p_fref[1][a->l1.bi16x16.i_ref][0], h->mb.pic.i_stride[0],
                                h->mb.bipred_weight[a->l0.bi16x16.i_ref][a->l1.bi16x16.i_ref] );
        cost00 = h->pixf.mbcmp[PIXEL_16x16]( h->mb.pic.p_fenc[0], FENC_STRIDE, pix0, 16 )
                   + ref_costs + l0_mv_cost + l1_mv_cost;

        if( h->mb.b_chroma_me )
        {
            ALIGNED_ARRAY_16( pixel, pixuv, [2],[8*FENC_STRIDE] );
            ALIGNED_ARRAY_16( pixel, bi, [8*FENC_STRIDE] );

            if( h->mb.b_interlaced & a->l0.bi16x16.i_ref )
            {
                int l0_mvy_offset = h->mb.b_interlaced & a->l0.bi16x16.i_ref ? (h->mb.i_mb_y & 1)*4 - 2 : 0;
                h->mc.mc_chroma( pixuv[0], pixuv[0]+8, FENC_STRIDE, h->mb.pic.p_fref[0][a->l0.bi16x16.i_ref][4],
                                 h->mb.pic.i_stride[1], 0, 0 + l0_mvy_offset, 8, 8 );
            }
            else
                h->mc.load_deinterleave_8x8x2_fenc( pixuv[0], h->mb.pic.p_fref[0][a->l0.bi16x16.i_ref][4], h->mb.pic.i_stride[1] );

            if( h->mb.b_interlaced & a->l1.bi16x16.i_ref )
            {
                int l1_mvy_offset = h->mb.b_interlaced & a->l1.bi16x16.i_ref ? (h->mb.i_mb_y & 1)*4 - 2 : 0;
                h->mc.mc_chroma( pixuv[1], pixuv[1]+8, FENC_STRIDE, h->mb.pic.p_fref[1][a->l1.bi16x16.i_ref][4],
                                 h->mb.pic.i_stride[1], 0, 0 + l1_mvy_offset, 8, 8 );
            }
            else
                h->mc.load_deinterleave_8x8x2_fenc( pixuv[1], h->mb.pic.p_fref[1][a->l1.bi16x16.i_ref][4], h->mb.pic.i_stride[1] );

            h->mc.avg[PIXEL_8x8]( bi, FENC_STRIDE, pixuv[0], FENC_STRIDE, pixuv[1], FENC_STRIDE,
                                  h->mb.bipred_weight[a->l0.bi16x16.i_ref][a->l1.bi16x16.i_ref] );
            h->mc.avg[PIXEL_8x8]( bi+8, FENC_STRIDE, pixuv[0]+8, FENC_STRIDE, pixuv[1]+8, FENC_STRIDE,
                                  h->mb.bipred_weight[a->l0.bi16x16.i_ref][a->l1.bi16x16.i_ref] );

            cost00 += h->pixf.mbcmp[PIXEL_8x8]( h->mb.pic.p_fenc[1], FENC_STRIDE, bi, FENC_STRIDE )
                   +  h->pixf.mbcmp[PIXEL_8x8]( h->mb.pic.p_fenc[2], FENC_STRIDE, bi+8, FENC_STRIDE );
        }

        if( cost00 < a->i_cost16x16bi )
        {
            M32( a->l0.bi16x16.mv ) = 0;
            M32( a->l1.bi16x16.mv ) = 0;
            a->l0.bi16x16.cost_mv = l0_mv_cost;
            a->l1.bi16x16.cost_mv = l1_mv_cost;
            a->i_cost16x16bi = cost00;
        }
    }

    /* mb type cost */
    a->i_cost16x16bi   += a->i_lambda * i_mb_b_cost_table[B_BI_BI];
    a->l0.me16x16.cost += a->i_lambda * i_mb_b_cost_table[B_L0_L0];
    a->l1.me16x16.cost += a->i_lambda * i_mb_b_cost_table[B_L1_L1];
}
//} mb_analyse_inter_b16x16 //} mb_analyse_inter_b16x16 //} mb_analyse_inter_b16x16 //} mb_analyse_inter_b16x16

//{ mb_analyse_inter_b8x8 //{ mb_analyse_inter_b8x8 //{ mb_analyse_inter_b8x8 //{ mb_analyse_inter_b8x8
static void dull_mb_analyse_inter_b8x8( x264_t *h, x264_mb_analysis_t *a )
{
    pixel **p_fref[2] =
        { h->mb.pic.p_fref[0][a->l0.me16x16.i_ref],
          h->mb.pic.p_fref[1][a->l1.me16x16.i_ref] };
    ALIGNED_ARRAY_8( pixel, pix,[2],[8*8] );
    int i, l;

    /* XXX Needed for x264_mb_predict_mv */
    h->mb.i_partition = D_8x8;

    a->i_cost8x8bi = 0;

    for( i = 0; i < 4; i++ )
    {
        int x8 = i&1;
        int y8 = i>>1;
        int i_part_cost;
        int i_part_cost_bi = 0;
        int stride[2] = {8,8};
        pixel *src[2];

        for( l = 0; l < 2; l++ )
        {
            x264_mb_analysis_list_t *lX = l ? &a->l1 : &a->l0;
            x264_me_t *m = &lX->me8x8[i];
            m->i_pixel = PIXEL_8x8;
            LOAD_FENC( m, h->mb.pic.p_fenc, 8*x8, 8*y8 );

            m->i_ref_cost = REF_COST( l, lX->me16x16.i_ref );
            m->i_ref = lX->me16x16.i_ref;

            LOAD_HPELS( m, p_fref[l], l, lX->me16x16.i_ref, 8*x8, 8*y8 );

            x264_macroblock_cache_ref( h, x8*2, y8*2, 2, 2, l, lX->me16x16.i_ref );
            x264_mb_predict_mv( h, l, 4*i, 2, m->mvp );
            x264_me_search( h, m, &lX->me16x16.mv, 1 );
            a->i_satd8x8[l][i] = m->cost - m->cost_mv;
            m->cost += m->i_ref_cost;

            x264_macroblock_cache_mv_ptr( h, 2*x8, 2*y8, 2, 2, l, m->mv );

            /* save mv for predicting other partitions within this MB */
            CP32( lX->mvc[lX->me16x16.i_ref][i+1], m->mv );

            /* BI mode */
            src[l] = h->mc.get_ref( pix[l], &stride[l], m->p_fref, m->i_stride[0],
                                    m->mv[0], m->mv[1], 8, 8, weight_none );
            i_part_cost_bi += m->cost_mv + m->i_ref_cost;
        }
        h->mc.avg[PIXEL_8x8]( pix[0], 8, src[0], stride[0], src[1], stride[1], h->mb.bipred_weight[a->l0.me16x16.i_ref][a->l1.me16x16.i_ref] );
        a->i_satd8x8[2][i] = h->pixf.mbcmp[PIXEL_8x8]( a->l0.me8x8[i].p_fenc[0], FENC_STRIDE, pix[0], 8 );
        i_part_cost_bi += a->i_satd8x8[2][i] + a->i_lambda * i_sub_mb_b_cost_table[D_BI_8x8];
        a->l0.me8x8[i].cost += a->i_lambda * i_sub_mb_b_cost_table[D_L0_8x8];
        a->l1.me8x8[i].cost += a->i_lambda * i_sub_mb_b_cost_table[D_L1_8x8];

        if( h->mb.b_chroma_me )
        {
            int i_chroma_cost = x264_analyse_bi_chroma( h, a, i, PIXEL_8x8 );
            i_part_cost_bi += i_chroma_cost;
            a->i_satd8x8[2][i] += i_chroma_cost;
        }

        i_part_cost = a->l0.me8x8[i].cost;
        h->mb.i_sub_partition[i] = D_L0_8x8;
        COPY2_IF_LT( i_part_cost, a->l1.me8x8[i].cost, h->mb.i_sub_partition[i], D_L1_8x8 );
        COPY2_IF_LT( i_part_cost, i_part_cost_bi, h->mb.i_sub_partition[i], D_BI_8x8 );
        COPY2_IF_LT( i_part_cost, a->i_cost8x8direct[i], h->mb.i_sub_partition[i], D_DIRECT_8x8 );
        a->i_cost8x8bi += i_part_cost;

        /* XXX Needed for x264_mb_predict_mv */
        x264_mb_cache_mv_b8x8( h, a, i, 0 );
    }

    /* mb type cost */
    a->i_cost8x8bi += a->i_lambda * i_mb_b_cost_table[B_8x8];
}
//} mb_analyse_inter_b8x8 //} mb_analyse_inter_b8x8 //} mb_analyse_inter_b8x8 //} mb_analyse_inter_b8x8

//{ macroblock_analyse //{ macroblock_analyse //{ macroblock_analyse //{ macroblock_analyse 
/*****************************************************************************
 * x264_macroblock_analyse:
 *****************************************************************************/
void dull_macroblock_analyse( x264_t *h )
{
    x264_mb_analysis_t analysis;
    int i_cost = COST_MAX;
    int i;

    h->mb.i_qp = x264_ratecontrol_mb_qp( h );
    /* If the QP of this MB is within 1 of the previous MB, code the same QP as the previous MB,
     * to lower the bit cost of the qp_delta.  Don't do this if QPRD is enabled. */
    if( h->param.rc.i_aq_mode && h->param.analyse.i_subpel_refine < 10 && abs(h->mb.i_qp - h->mb.i_last_qp) == 1 )
        h->mb.i_qp = h->mb.i_last_qp;

    dull_mb_analyse_init( h, &analysis );

    /*--------------------------- Do the analysis ---------------------------*/
    if( h->sh.i_type == SLICE_TYPE_I )
    {
intra_analysis:
        if( analysis.i_mbrd )
            x264_mb_init_fenc_cache( h, analysis.i_mbrd >= 2 );
        x264_mb_analyse_intra( h, &analysis, COST_MAX );
        if( analysis.i_mbrd )
            x264_intra_rd( h, &analysis, COST_MAX );

        i_cost = analysis.i_satd_i16x16;
        h->mb.i_type = I_16x16;
        COPY2_IF_LT( i_cost, analysis.i_satd_i4x4, h->mb.i_type, I_4x4 );
        COPY2_IF_LT( i_cost, analysis.i_satd_i8x8, h->mb.i_type, I_8x8 );
        if( analysis.i_satd_pcm < i_cost )
            h->mb.i_type = I_PCM;

        else if( analysis.i_mbrd >= 2 )
            x264_intra_rd_refine( h, &analysis );
    }
    else if( h->sh.i_type == SLICE_TYPE_P )
//{ macroblock_analyse_P //{ macroblock_analyse_P //{ macroblock_analyse_P //{ macroblock_analyse_P 
    {
        int b_skip = 0;

        analysis.b_try_skip = 0;
        if( analysis.b_force_intra )
        {
            if( !h->param.analyse.b_psy )
            {
                x264_mb_analyse_init_qp( h, &analysis, X264_MAX( h->mb.i_qp - h->mb.ip_offset, h->param.rc.i_qp_min ) );
                goto intra_analysis;
            }
        }
        else
        {
            /* Fast P_SKIP detection */
            if( h->param.analyse.b_fast_pskip )
            {
                if( h->i_thread_frames > 1 && h->mb.cache.pskip_mv[1] > h->mb.mv_max_spel[1] )
                    // FIXME don't need to check this if the reference frame is done
                    {}
                else if( h->param.analyse.i_subpel_refine >= 3 )
                    analysis.b_try_skip = 1;
                else if( h->mb.i_mb_type_left == P_SKIP ||
                         h->mb.i_mb_type_top == P_SKIP ||
                         h->mb.i_mb_type_topleft == P_SKIP ||
                         h->mb.i_mb_type_topright == P_SKIP )
                    b_skip = x264_macroblock_probe_pskip( h );
            }
        }

        if( b_skip )
        {
            h->mb.i_type = P_SKIP;
            h->mb.i_partition = D_16x16;
            assert( h->mb.cache.pskip_mv[1] <= h->mb.mv_max_spel[1] || h->i_thread_frames == 1 );
            /* Set up MVs for future predictors */
            for( i = 0; i < h->mb.pic.i_fref[0]; i++ )
                M32( h->mb.mvr[0][i][h->mb.i_mb_xy] ) = 0;
        }
        else
        {
            const unsigned int flags = h->param.analyse.inter;
            int i_type;
            int i_partition;
            int i_thresh16x8;
            int i_satd_inter, i_satd_intra;

            x264_mb_analyse_load_costs( h, &analysis );

            dull_mb_analyse_inter_p16x16( h, &analysis );

            if( h->mb.i_type == P_SKIP )
            {
                for( i = 1; i < h->mb.pic.i_fref[0]; i++ )
                    M32( h->mb.mvr[0][i][h->mb.i_mb_xy] ) = 0;
                return;
            }

            if( flags & X264_ANALYSE_PSUB16x16 )
            {
                if( h->param.analyse.b_mixed_references )
                    x264_mb_analyse_inter_p8x8_mixed_ref( h, &analysis );
                else
                    dull_mb_analyse_inter_p8x8( h, &analysis );
            }

            /* Select best inter mode */
            i_type = P_L0;
            i_partition = D_16x16;
            i_cost = analysis.l0.me16x16.cost;

            if( ( flags & X264_ANALYSE_PSUB16x16 ) &&
                analysis.l0.i_cost8x8 < analysis.l0.me16x16.cost )
            {
                i_type = P_8x8;
                i_partition = D_8x8;
                i_cost = analysis.l0.i_cost8x8;

                /* Do sub 8x8 */
                if( flags & X264_ANALYSE_PSUB8x8 )
                {
                    for( i = 0; i < 4; i++ )
                    {
                        x264_mb_analyse_inter_p4x4( h, &analysis, i );
                        if( analysis.l0.i_cost4x4[i] < analysis.l0.me8x8[i].cost )
                        {
                            int i_cost8x8 = analysis.l0.i_cost4x4[i];
                            h->mb.i_sub_partition[i] = D_L0_4x4;

                            x264_mb_analyse_inter_p8x4( h, &analysis, i );
                            COPY2_IF_LT( i_cost8x8, analysis.l0.i_cost8x4[i],
                                         h->mb.i_sub_partition[i], D_L0_8x4 );

                            x264_mb_analyse_inter_p4x8( h, &analysis, i );
                            COPY2_IF_LT( i_cost8x8, analysis.l0.i_cost4x8[i],
                                         h->mb.i_sub_partition[i], D_L0_4x8 );

                            i_cost += i_cost8x8 - analysis.l0.me8x8[i].cost;
                        }
                        x264_mb_cache_mv_p8x8( h, &analysis, i );
                    }
                    analysis.l0.i_cost8x8 = i_cost;
                }
            }

            /* Now do 16x8/8x16 */
            i_thresh16x8 = analysis.l0.me8x8[1].cost_mv + analysis.l0.me8x8[2].cost_mv;
            if( ( flags & X264_ANALYSE_PSUB16x16 ) &&
                analysis.l0.i_cost8x8 < analysis.l0.me16x16.cost + i_thresh16x8 )
            {
                int i_avg_mv_ref_cost = (analysis.l0.me8x8[2].cost_mv + analysis.l0.me8x8[2].i_ref_cost
                                      + analysis.l0.me8x8[3].cost_mv + analysis.l0.me8x8[3].i_ref_cost + 1) >> 1;
                analysis.i_cost_est16x8[1] = analysis.i_satd8x8[0][2] + analysis.i_satd8x8[0][3] + i_avg_mv_ref_cost;

                x264_mb_analyse_inter_p16x8( h, &analysis, i_cost );
                COPY3_IF_LT( i_cost, analysis.l0.i_cost16x8, i_type, P_L0, i_partition, D_16x8 );

                i_avg_mv_ref_cost = (analysis.l0.me8x8[1].cost_mv + analysis.l0.me8x8[1].i_ref_cost
                                  + analysis.l0.me8x8[3].cost_mv + analysis.l0.me8x8[3].i_ref_cost + 1) >> 1;
                analysis.i_cost_est8x16[1] = analysis.i_satd8x8[0][1] + analysis.i_satd8x8[0][3] + i_avg_mv_ref_cost;

                x264_mb_analyse_inter_p8x16( h, &analysis, i_cost );
                COPY3_IF_LT( i_cost, analysis.l0.i_cost8x16, i_type, P_L0, i_partition, D_8x16 );
            }

            h->mb.i_partition = i_partition;

            /* refine qpel */
            //FIXME mb_type costs?
            if( analysis.i_mbrd || !h->mb.i_subpel_refine )
            {
                /* refine later */
            }
            else if( i_partition == D_16x16 )
            {
                x264_me_refine_qpel( h, &analysis.l0.me16x16 );
                i_cost = analysis.l0.me16x16.cost;
            }
            else if( i_partition == D_16x8 )
            {
                x264_me_refine_qpel( h, &analysis.l0.me16x8[0] );
                x264_me_refine_qpel( h, &analysis.l0.me16x8[1] );
                i_cost = analysis.l0.me16x8[0].cost + analysis.l0.me16x8[1].cost;
            }
            else if( i_partition == D_8x16 )
            {
                x264_me_refine_qpel( h, &analysis.l0.me8x16[0] );
                x264_me_refine_qpel( h, &analysis.l0.me8x16[1] );
                i_cost = analysis.l0.me8x16[0].cost + analysis.l0.me8x16[1].cost;
            }
            else if( i_partition == D_8x8 )
            {
                int i8x8;
                i_cost = 0;
                for( i8x8 = 0; i8x8 < 4; i8x8++ )
                {
                    switch( h->mb.i_sub_partition[i8x8] )
                    {
                        case D_L0_8x8:
                            x264_me_refine_qpel( h, &analysis.l0.me8x8[i8x8] );
                            i_cost += analysis.l0.me8x8[i8x8].cost;
                            break;
                        case D_L0_8x4:
                            x264_me_refine_qpel( h, &analysis.l0.me8x4[i8x8][0] );
                            x264_me_refine_qpel( h, &analysis.l0.me8x4[i8x8][1] );
                            i_cost += analysis.l0.me8x4[i8x8][0].cost +
                                      analysis.l0.me8x4[i8x8][1].cost;
                            break;
                        case D_L0_4x8:
                            x264_me_refine_qpel( h, &analysis.l0.me4x8[i8x8][0] );
                            x264_me_refine_qpel( h, &analysis.l0.me4x8[i8x8][1] );
                            i_cost += analysis.l0.me4x8[i8x8][0].cost +
                                      analysis.l0.me4x8[i8x8][1].cost;
                            break;

                        case D_L0_4x4:
                            x264_me_refine_qpel( h, &analysis.l0.me4x4[i8x8][0] );
                            x264_me_refine_qpel( h, &analysis.l0.me4x4[i8x8][1] );
                            x264_me_refine_qpel( h, &analysis.l0.me4x4[i8x8][2] );
                            x264_me_refine_qpel( h, &analysis.l0.me4x4[i8x8][3] );
                            i_cost += analysis.l0.me4x4[i8x8][0].cost +
                                      analysis.l0.me4x4[i8x8][1].cost +
                                      analysis.l0.me4x4[i8x8][2].cost +
                                      analysis.l0.me4x4[i8x8][3].cost;
                            break;
                        default:
                            x264_log( h, X264_LOG_ERROR, "internal error (!8x8 && !4x4)\n" );
                            break;
                    }
                }
            }

            if( h->mb.b_chroma_me )
            {
                x264_mb_analyse_intra_chroma( h, &analysis );
                x264_mb_analyse_intra( h, &analysis, i_cost - analysis.i_satd_i8x8chroma );
                analysis.i_satd_i16x16 += analysis.i_satd_i8x8chroma;
                analysis.i_satd_i8x8 += analysis.i_satd_i8x8chroma;
                analysis.i_satd_i4x4 += analysis.i_satd_i8x8chroma;
            }
            else
                x264_mb_analyse_intra( h, &analysis, i_cost );

            i_satd_inter = i_cost;
            i_satd_intra = X264_MIN3( analysis.i_satd_i16x16,
                                      analysis.i_satd_i8x8,
                                      analysis.i_satd_i4x4 );

            if( analysis.i_mbrd )
            {
                x264_mb_analyse_p_rd( h, &analysis, X264_MIN(i_satd_inter, i_satd_intra) );
                i_type = P_L0;
                i_partition = D_16x16;
                i_cost = analysis.l0.i_rd16x16;
                COPY2_IF_LT( i_cost, analysis.l0.i_cost16x8, i_partition, D_16x8 );
                COPY2_IF_LT( i_cost, analysis.l0.i_cost8x16, i_partition, D_8x16 );
                COPY3_IF_LT( i_cost, analysis.l0.i_cost8x8, i_partition, D_8x8, i_type, P_8x8 );
                h->mb.i_type = i_type;
                h->mb.i_partition = i_partition;
                if( i_cost < COST_MAX )
                    x264_mb_analyse_transform_rd( h, &analysis, &i_satd_inter, &i_cost );
                x264_intra_rd( h, &analysis, i_satd_inter * 5/4 );
            }

            COPY2_IF_LT( i_cost, analysis.i_satd_i16x16, i_type, I_16x16 );
            COPY2_IF_LT( i_cost, analysis.i_satd_i8x8, i_type, I_8x8 );
            COPY2_IF_LT( i_cost, analysis.i_satd_i4x4, i_type, I_4x4 );
            COPY2_IF_LT( i_cost, analysis.i_satd_pcm, i_type, I_PCM );

            h->mb.i_type = i_type;

            if( analysis.b_force_intra && !IS_INTRA(i_type) )
            {
                /* Intra masking: copy fdec to fenc and re-encode the block as intra in order to make it appear as if
                 * it was an inter block. */
                x264_analyse_update_cache( h, &analysis );
                x264_macroblock_encode( h );
                h->mc.copy[PIXEL_16x16]( h->mb.pic.p_fenc[0], FENC_STRIDE, h->mb.pic.p_fdec[0], FDEC_STRIDE, 16 );
                h->mc.copy[PIXEL_8x8]  ( h->mb.pic.p_fenc[1], FENC_STRIDE, h->mb.pic.p_fdec[1], FDEC_STRIDE, 8 );
                h->mc.copy[PIXEL_8x8]  ( h->mb.pic.p_fenc[2], FENC_STRIDE, h->mb.pic.p_fdec[2], FDEC_STRIDE, 8 );
                x264_mb_analyse_init_qp( h, &analysis, X264_MAX( h->mb.i_qp - h->mb.ip_offset, h->param.rc.i_qp_min ) );
                goto intra_analysis;
            }

            if( analysis.i_mbrd >= 2 && h->mb.i_type != I_PCM )
            {
                if( IS_INTRA( h->mb.i_type ) )
                {
                    x264_intra_rd_refine( h, &analysis );
                }
                else if( i_partition == D_16x16 )
                {
                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, analysis.l0.me16x16.i_ref );
                    analysis.l0.me16x16.cost = i_cost;
                    x264_me_refine_qpel_rd( h, &analysis.l0.me16x16, analysis.i_lambda2, 0, 0 );
                }
                else if( i_partition == D_16x8 )
                {
                    h->mb.i_sub_partition[0] = h->mb.i_sub_partition[1] =
                    h->mb.i_sub_partition[2] = h->mb.i_sub_partition[3] = D_L0_8x8;
                    x264_macroblock_cache_ref( h, 0, 0, 4, 2, 0, analysis.l0.me16x8[0].i_ref );
                    x264_macroblock_cache_ref( h, 0, 2, 4, 2, 0, analysis.l0.me16x8[1].i_ref );
                    x264_me_refine_qpel_rd( h, &analysis.l0.me16x8[0], analysis.i_lambda2, 0, 0 );
                    x264_me_refine_qpel_rd( h, &analysis.l0.me16x8[1], analysis.i_lambda2, 8, 0 );
                }
                else if( i_partition == D_8x16 )
                {
                    h->mb.i_sub_partition[0] = h->mb.i_sub_partition[1] =
                    h->mb.i_sub_partition[2] = h->mb.i_sub_partition[3] = D_L0_8x8;
                    x264_macroblock_cache_ref( h, 0, 0, 2, 4, 0, analysis.l0.me8x16[0].i_ref );
                    x264_macroblock_cache_ref( h, 2, 0, 2, 4, 0, analysis.l0.me8x16[1].i_ref );
                    x264_me_refine_qpel_rd( h, &analysis.l0.me8x16[0], analysis.i_lambda2, 0, 0 );
                    x264_me_refine_qpel_rd( h, &analysis.l0.me8x16[1], analysis.i_lambda2, 4, 0 );
                }
                else if( i_partition == D_8x8 )
                {
                    int i8x8;
                    x264_analyse_update_cache( h, &analysis );
                    for( i8x8 = 0; i8x8 < 4; i8x8++ )
                    {
                        if( h->mb.i_sub_partition[i8x8] == D_L0_8x8 )
                        {
                            x264_me_refine_qpel_rd( h, &analysis.l0.me8x8[i8x8], analysis.i_lambda2, i8x8*4, 0 );
                        }
                        else if( h->mb.i_sub_partition[i8x8] == D_L0_8x4 )
                        {
                            x264_me_refine_qpel_rd( h, &analysis.l0.me8x4[i8x8][0], analysis.i_lambda2, i8x8*4+0, 0 );
                            x264_me_refine_qpel_rd( h, &analysis.l0.me8x4[i8x8][1], analysis.i_lambda2, i8x8*4+2, 0 );
                        }
                        else if( h->mb.i_sub_partition[i8x8] == D_L0_4x8 )
                        {
                            x264_me_refine_qpel_rd( h, &analysis.l0.me4x8[i8x8][0], analysis.i_lambda2, i8x8*4+0, 0 );
                            x264_me_refine_qpel_rd( h, &analysis.l0.me4x8[i8x8][1], analysis.i_lambda2, i8x8*4+1, 0 );
                        }
                        else if( h->mb.i_sub_partition[i8x8] == D_L0_4x4 )
                        {
                            x264_me_refine_qpel_rd( h, &analysis.l0.me4x4[i8x8][0], analysis.i_lambda2, i8x8*4+0, 0 );
                            x264_me_refine_qpel_rd( h, &analysis.l0.me4x4[i8x8][1], analysis.i_lambda2, i8x8*4+1, 0 );
                            x264_me_refine_qpel_rd( h, &analysis.l0.me4x4[i8x8][2], analysis.i_lambda2, i8x8*4+2, 0 );
                            x264_me_refine_qpel_rd( h, &analysis.l0.me4x4[i8x8][3], analysis.i_lambda2, i8x8*4+3, 0 );
                        }
                    }
                }
            }
        }
    }
//} macroblock_analyse_P //} macroblock_analyse_P //} macroblock_analyse_P //} macroblock_analyse_P 
    else if( h->sh.i_type == SLICE_TYPE_B )
//{ macroblock_analyse_B //{ macroblock_analyse_B //{ macroblock_analyse_B //{ macroblock_analyse_B
    {
        int i_bskip_cost = COST_MAX;
        int b_skip = 0;

        if( analysis.i_mbrd )
            x264_mb_init_fenc_cache( h, analysis.i_mbrd >= 2 );

        h->mb.i_type = B_SKIP;
        if( h->mb.b_direct_auto_write )
        {
            /* direct=auto heuristic: prefer whichever mode allows more Skip macroblocks */
            for( i = 0; i < 2; i++ )
            {
                int b_changed = 1;
                h->sh.b_direct_spatial_mv_pred ^= 1;
                analysis.b_direct_available = x264_mb_predict_mv_direct16x16( h, i && analysis.b_direct_available ? &b_changed : NULL );
                if( analysis.b_direct_available )
                {
                    if( b_changed )
                    {
                        x264_mb_mc( h );
                        b_skip = x264_macroblock_probe_bskip( h );
                    }
                    h->stat.frame.i_direct_score[ h->sh.b_direct_spatial_mv_pred ] += b_skip;
                }
                else
                    b_skip = 0;
            }
        }
        else
            analysis.b_direct_available = x264_mb_predict_mv_direct16x16( h, NULL );

        analysis.b_try_skip = 0;
        if( analysis.b_direct_available )
        {
            if( !h->mb.b_direct_auto_write )
                x264_mb_mc( h );
            if( analysis.i_mbrd )
            {
                i_bskip_cost = ssd_mb( h );
                /* 6 = minimum cavlc cost of a non-skipped MB */
                b_skip = h->mb.b_skip_mc = i_bskip_cost <= ((6 * analysis.i_lambda2 + 128) >> 8);
            }
            else if( !h->mb.b_direct_auto_write )
            {
                /* Conditioning the probe on neighboring block types
                 * doesn't seem to help speed or quality. */
                analysis.b_try_skip = x264_macroblock_probe_bskip( h );
                if( h->param.analyse.i_subpel_refine < 3 )
                    b_skip = analysis.b_try_skip;
            }
            /* Set up MVs for future predictors */
            if( b_skip )
            {
                for( i = 0; i < h->mb.pic.i_fref[0]; i++ )
                    M32( h->mb.mvr[0][i][h->mb.i_mb_xy] ) = 0;
                for( i = 0; i < h->mb.pic.i_fref[1]; i++ )
                    M32( h->mb.mvr[1][i][h->mb.i_mb_xy] ) = 0;
            }
        }

        if( !b_skip )
        {
            const unsigned int flags = h->param.analyse.inter;
            int i_type;
            int i_partition;
            int i_satd_inter;
            h->mb.b_skip_mc = 0;
            h->mb.i_type = B_DIRECT;

            x264_mb_analyse_load_costs( h, &analysis );

            /* select best inter mode */
            /* direct must be first */
            if( analysis.b_direct_available )
                x264_mb_analyse_inter_direct( h, &analysis );

            dull_mb_analyse_inter_b16x16( h, &analysis );

            if( h->mb.i_type == B_SKIP )
            {
                for( i = 1; i < h->mb.pic.i_fref[0]; i++ )
                    M32( h->mb.mvr[0][i][h->mb.i_mb_xy] ) = 0;
                for( i = 1; i < h->mb.pic.i_fref[1]; i++ )
                    M32( h->mb.mvr[0][i][h->mb.i_mb_xy] ) = 0;
                return;
            }

            i_type = B_L0_L0;
            i_partition = D_16x16;
            i_cost = analysis.l0.me16x16.cost;
            COPY2_IF_LT( i_cost, analysis.l1.me16x16.cost, i_type, B_L1_L1 );
            COPY2_IF_LT( i_cost, analysis.i_cost16x16bi, i_type, B_BI_BI );
            COPY2_IF_LT( i_cost, analysis.i_cost16x16direct, i_type, B_DIRECT );

            if( analysis.i_mbrd && analysis.i_cost16x16direct <= i_cost * 33/32 )
            {
                x264_mb_analyse_b_rd( h, &analysis, i_cost );
                if( i_bskip_cost < analysis.i_rd16x16direct &&
                    i_bskip_cost < analysis.i_rd16x16bi &&
                    i_bskip_cost < analysis.l0.i_rd16x16 &&
                    i_bskip_cost < analysis.l1.i_rd16x16 )
                {
                    h->mb.i_type = B_SKIP;
                    x264_analyse_update_cache( h, &analysis );
                    return;
                }
            }

            if( flags & X264_ANALYSE_BSUB16x16 )
            {
                int i_cost_est16x8bi_total = 0, i_cost_est8x16bi_total = 0, i, try_16x8_first;
                int i_mb_type, i_partition16x8[2], i_partition8x16[2];
                if( h->param.analyse.b_mixed_references )
                    x264_mb_analyse_inter_b8x8_mixed_ref( h, &analysis );
                else
                    x264_mb_analyse_inter_b8x8( h, &analysis );

                COPY3_IF_LT( i_cost, analysis.i_cost8x8bi, i_type, B_8x8, i_partition, D_8x8 );

                /* Try to estimate the cost of b16x8/b8x16 based on the satd scores of the b8x8 modes */
                for( i = 0; i < 2; i++ )
                {
                    int avg_l0_mv_ref_cost, avg_l1_mv_ref_cost;
                    int i_l0_satd, i_l1_satd, i_bi_satd, i_best_cost;
                    // 16x8
                    i_best_cost = COST_MAX;
                    i_l0_satd = analysis.i_satd8x8[0][i*2] + analysis.i_satd8x8[0][i*2+1];
                    i_l1_satd = analysis.i_satd8x8[1][i*2] + analysis.i_satd8x8[1][i*2+1];
                    i_bi_satd = analysis.i_satd8x8[2][i*2] + analysis.i_satd8x8[2][i*2+1];
                    avg_l0_mv_ref_cost = ( analysis.l0.me8x8[i*2].cost_mv + analysis.l0.me8x8[i*2].i_ref_cost
                                         + analysis.l0.me8x8[i*2+1].cost_mv + analysis.l0.me8x8[i*2+1].i_ref_cost + 1 ) >> 1;
                    avg_l1_mv_ref_cost = ( analysis.l1.me8x8[i*2].cost_mv + analysis.l1.me8x8[i*2].i_ref_cost
                                         + analysis.l1.me8x8[i*2+1].cost_mv + analysis.l1.me8x8[i*2+1].i_ref_cost + 1 ) >> 1;
                    COPY2_IF_LT( i_best_cost, i_l0_satd + avg_l0_mv_ref_cost, i_partition16x8[i], D_L0_8x8 );
                    COPY2_IF_LT( i_best_cost, i_l1_satd + avg_l1_mv_ref_cost, i_partition16x8[i], D_L1_8x8 );
                    COPY2_IF_LT( i_best_cost, i_bi_satd + avg_l0_mv_ref_cost + avg_l1_mv_ref_cost, i_partition16x8[i], D_BI_8x8 );
                    analysis.i_cost_est16x8[i] = i_best_cost;

                    // 8x16
                    i_best_cost = COST_MAX;
                    i_l0_satd = analysis.i_satd8x8[0][i] + analysis.i_satd8x8[0][i+2];
                    i_l1_satd = analysis.i_satd8x8[1][i] + analysis.i_satd8x8[1][i+2];
                    i_bi_satd = analysis.i_satd8x8[2][i] + analysis.i_satd8x8[2][i+2];
                    avg_l0_mv_ref_cost = ( analysis.l0.me8x8[i].cost_mv + analysis.l0.me8x8[i].i_ref_cost
                                         + analysis.l0.me8x8[i+2].cost_mv + analysis.l0.me8x8[i+2].i_ref_cost + 1 ) >> 1;
                    avg_l1_mv_ref_cost = ( analysis.l1.me8x8[i].cost_mv + analysis.l1.me8x8[i].i_ref_cost
                                         + analysis.l1.me8x8[i+2].cost_mv + analysis.l1.me8x8[i+2].i_ref_cost + 1 ) >> 1;
                    COPY2_IF_LT( i_best_cost, i_l0_satd + avg_l0_mv_ref_cost, i_partition8x16[i], D_L0_8x8 );
                    COPY2_IF_LT( i_best_cost, i_l1_satd + avg_l1_mv_ref_cost, i_partition8x16[i], D_L1_8x8 );
                    COPY2_IF_LT( i_best_cost, i_bi_satd + avg_l0_mv_ref_cost + avg_l1_mv_ref_cost, i_partition8x16[i], D_BI_8x8 );
                    analysis.i_cost_est8x16[i] = i_best_cost;
                }
                i_mb_type = B_L0_L0 + (i_partition16x8[0]>>2) * 3 + (i_partition16x8[1]>>2);
                analysis.i_cost_est16x8[1] += analysis.i_lambda * i_mb_b16x8_cost_table[i_mb_type];
                i_cost_est16x8bi_total = analysis.i_cost_est16x8[0] + analysis.i_cost_est16x8[1];
                i_mb_type = B_L0_L0 + (i_partition8x16[0]>>2) * 3 + (i_partition8x16[1]>>2);
                analysis.i_cost_est8x16[1] += analysis.i_lambda * i_mb_b16x8_cost_table[i_mb_type];
                i_cost_est8x16bi_total = analysis.i_cost_est8x16[0] + analysis.i_cost_est8x16[1];

                /* We can gain a little speed by checking the mode with the lowest estimated cost first */
                try_16x8_first = i_cost_est16x8bi_total < i_cost_est8x16bi_total;
                if( try_16x8_first && i_cost_est16x8bi_total < i_cost )
                {
                    x264_mb_analyse_inter_b16x8( h, &analysis, i_cost );
                    COPY3_IF_LT( i_cost, analysis.i_cost16x8bi, i_type, analysis.i_mb_type16x8, i_partition, D_16x8 );
                }
                if( i_cost_est8x16bi_total < i_cost )
                {
                    x264_mb_analyse_inter_b8x16( h, &analysis, i_cost );
                    COPY3_IF_LT( i_cost, analysis.i_cost8x16bi, i_type, analysis.i_mb_type8x16, i_partition, D_8x16 );
                }
                if( !try_16x8_first && i_cost_est16x8bi_total < i_cost )
                {
                    x264_mb_analyse_inter_b16x8( h, &analysis, i_cost );
                    COPY3_IF_LT( i_cost, analysis.i_cost16x8bi, i_type, analysis.i_mb_type16x8, i_partition, D_16x8 );
                }
            }

            if( analysis.i_mbrd || !h->mb.i_subpel_refine )
            {
                /* refine later */
            }
            /* refine qpel */
            else if( i_partition == D_16x16 )
            {
                analysis.l0.me16x16.cost -= analysis.i_lambda * i_mb_b_cost_table[B_L0_L0];
                analysis.l1.me16x16.cost -= analysis.i_lambda * i_mb_b_cost_table[B_L1_L1];
                if( i_type == B_L0_L0 )
                {
                    x264_me_refine_qpel( h, &analysis.l0.me16x16 );
                    i_cost = analysis.l0.me16x16.cost
                           + analysis.i_lambda * i_mb_b_cost_table[B_L0_L0];
                }
                else if( i_type == B_L1_L1 )
                {
                    x264_me_refine_qpel( h, &analysis.l1.me16x16 );
                    i_cost = analysis.l1.me16x16.cost
                           + analysis.i_lambda * i_mb_b_cost_table[B_L1_L1];
                }
                else if( i_type == B_BI_BI )
                {
                    x264_me_refine_qpel( h, &analysis.l0.bi16x16 );
                    x264_me_refine_qpel( h, &analysis.l1.bi16x16 );
                }
            }
            else if( i_partition == D_16x8 )
            {
                for( i = 0; i < 2; i++ )
                {
                    if( analysis.i_mb_partition16x8[i] != D_L1_8x8 )
                        x264_me_refine_qpel( h, &analysis.l0.me16x8[i] );
                    if( analysis.i_mb_partition16x8[i] != D_L0_8x8 )
                        x264_me_refine_qpel( h, &analysis.l1.me16x8[i] );
                }
            }
            else if( i_partition == D_8x16 )
            {
                for( i = 0; i < 2; i++ )
                {
                    if( analysis.i_mb_partition8x16[i] != D_L1_8x8 )
                        x264_me_refine_qpel( h, &analysis.l0.me8x16[i] );
                    if( analysis.i_mb_partition8x16[i] != D_L0_8x8 )
                        x264_me_refine_qpel( h, &analysis.l1.me8x16[i] );
                }
            }
            else if( i_partition == D_8x8 )
            {
                for( i = 0; i < 4; i++ )
                {
                    x264_me_t *m;
                    int i_part_cost_old;
                    int i_type_cost;
                    int i_part_type = h->mb.i_sub_partition[i];
                    int b_bidir = (i_part_type == D_BI_8x8);

                    if( i_part_type == D_DIRECT_8x8 )
                        continue;
                    if( x264_mb_partition_listX_table[0][i_part_type] )
                    {
                        m = &analysis.l0.me8x8[i];
                        i_part_cost_old = m->cost;
                        i_type_cost = analysis.i_lambda * i_sub_mb_b_cost_table[D_L0_8x8];
                        m->cost -= i_type_cost;
                        x264_me_refine_qpel( h, m );
                        if( !b_bidir )
                            analysis.i_cost8x8bi += m->cost + i_type_cost - i_part_cost_old;
                    }
                    if( x264_mb_partition_listX_table[1][i_part_type] )
                    {
                        m = &analysis.l1.me8x8[i];
                        i_part_cost_old = m->cost;
                        i_type_cost = analysis.i_lambda * i_sub_mb_b_cost_table[D_L1_8x8];
                        m->cost -= i_type_cost;
                        x264_me_refine_qpel( h, m );
                        if( !b_bidir )
                            analysis.i_cost8x8bi += m->cost + i_type_cost - i_part_cost_old;
                    }
                    /* TODO: update mvp? */
                }
            }

            i_satd_inter = i_cost;

            if( analysis.i_mbrd )
            {
                x264_mb_analyse_b_rd( h, &analysis, i_satd_inter );
                i_type = B_SKIP;
                i_cost = i_bskip_cost;
                i_partition = D_16x16;
                COPY2_IF_LT( i_cost, analysis.l0.i_rd16x16, i_type, B_L0_L0 );
                COPY2_IF_LT( i_cost, analysis.l1.i_rd16x16, i_type, B_L1_L1 );
                COPY2_IF_LT( i_cost, analysis.i_rd16x16bi, i_type, B_BI_BI );
                COPY2_IF_LT( i_cost, analysis.i_rd16x16direct, i_type, B_DIRECT );
                COPY3_IF_LT( i_cost, analysis.i_rd16x8bi, i_type, analysis.i_mb_type16x8, i_partition, D_16x8 );
                COPY3_IF_LT( i_cost, analysis.i_rd8x16bi, i_type, analysis.i_mb_type8x16, i_partition, D_8x16 );
                COPY3_IF_LT( i_cost, analysis.i_rd8x8bi, i_type, B_8x8, i_partition, D_8x8 );

                h->mb.i_type = i_type;
                h->mb.i_partition = i_partition;
            }

            if( h->mb.b_chroma_me )
            {
                x264_mb_analyse_intra_chroma( h, &analysis );
                x264_mb_analyse_intra( h, &analysis, i_satd_inter - analysis.i_satd_i8x8chroma );
                analysis.i_satd_i16x16 += analysis.i_satd_i8x8chroma;
                analysis.i_satd_i8x8 += analysis.i_satd_i8x8chroma;
                analysis.i_satd_i4x4 += analysis.i_satd_i8x8chroma;
            }
            else
                x264_mb_analyse_intra( h, &analysis, i_satd_inter );

            if( analysis.i_mbrd )
            {
                x264_mb_analyse_transform_rd( h, &analysis, &i_satd_inter, &i_cost );
                x264_intra_rd( h, &analysis, i_satd_inter * 17/16 );
            }

            COPY2_IF_LT( i_cost, analysis.i_satd_i16x16, i_type, I_16x16 );
            COPY2_IF_LT( i_cost, analysis.i_satd_i8x8, i_type, I_8x8 );
            COPY2_IF_LT( i_cost, analysis.i_satd_i4x4, i_type, I_4x4 );
            COPY2_IF_LT( i_cost, analysis.i_satd_pcm, i_type, I_PCM );

            h->mb.i_type = i_type;
            h->mb.i_partition = i_partition;

            if( analysis.i_mbrd >= 2 && IS_INTRA( i_type ) && i_type != I_PCM )
                x264_intra_rd_refine( h, &analysis );
            if( h->mb.i_subpel_refine >= 5 )
                x264_refine_bidir( h, &analysis );

            if( analysis.i_mbrd >= 2 && i_type > B_DIRECT && i_type < B_SKIP )
            {
                int i_biweight;
                x264_analyse_update_cache( h, &analysis );

                if( i_partition == D_16x16 )
                {
                    if( i_type == B_L0_L0 )
                    {
                        analysis.l0.me16x16.cost = i_cost;
                        x264_me_refine_qpel_rd( h, &analysis.l0.me16x16, analysis.i_lambda2, 0, 0 );
                    }
                    else if( i_type == B_L1_L1 )
                    {
                        analysis.l1.me16x16.cost = i_cost;
                        x264_me_refine_qpel_rd( h, &analysis.l1.me16x16, analysis.i_lambda2, 0, 1 );
                    }
                    else if( i_type == B_BI_BI )
                    {
                        i_biweight = h->mb.bipred_weight[analysis.l0.bi16x16.i_ref][analysis.l1.bi16x16.i_ref];
                        x264_me_refine_bidir_rd( h, &analysis.l0.bi16x16, &analysis.l1.bi16x16, i_biweight, 0, analysis.i_lambda2 );
                    }
                }
                else if( i_partition == D_16x8 )
                {
                    for( i = 0; i < 2; i++ )
                    {
                        h->mb.i_sub_partition[i*2] = h->mb.i_sub_partition[i*2+1] = analysis.i_mb_partition16x8[i];
                        if( analysis.i_mb_partition16x8[i] == D_L0_8x8 )
                            x264_me_refine_qpel_rd( h, &analysis.l0.me16x8[i], analysis.i_lambda2, i*8, 0 );
                        else if( analysis.i_mb_partition16x8[i] == D_L1_8x8 )
                            x264_me_refine_qpel_rd( h, &analysis.l1.me16x8[i], analysis.i_lambda2, i*8, 1 );
                        else if( analysis.i_mb_partition16x8[i] == D_BI_8x8 )
                        {
                            i_biweight = h->mb.bipred_weight[analysis.l0.me16x8[i].i_ref][analysis.l1.me16x8[i].i_ref];
                            x264_me_refine_bidir_rd( h, &analysis.l0.me16x8[i], &analysis.l1.me16x8[i], i_biweight, i*2, analysis.i_lambda2 );
                        }
                    }
                }
                else if( i_partition == D_8x16 )
                {
                    for( i = 0; i < 2; i++ )
                    {
                        h->mb.i_sub_partition[i] = h->mb.i_sub_partition[i+2] = analysis.i_mb_partition8x16[i];
                        if( analysis.i_mb_partition8x16[i] == D_L0_8x8 )
                            x264_me_refine_qpel_rd( h, &analysis.l0.me8x16[i], analysis.i_lambda2, i*4, 0 );
                        else if( analysis.i_mb_partition8x16[i] == D_L1_8x8 )
                            x264_me_refine_qpel_rd( h, &analysis.l1.me8x16[i], analysis.i_lambda2, i*4, 1 );
                        else if( analysis.i_mb_partition8x16[i] == D_BI_8x8 )
                        {
                            i_biweight = h->mb.bipred_weight[analysis.l0.me8x16[i].i_ref][analysis.l1.me8x16[i].i_ref];
                            x264_me_refine_bidir_rd( h, &analysis.l0.me8x16[i], &analysis.l1.me8x16[i], i_biweight, i, analysis.i_lambda2 );
                        }
                    }
                }
                else if( i_partition == D_8x8 )
                {
                    for( i = 0; i < 4; i++ )
                    {
                        if( h->mb.i_sub_partition[i] == D_L0_8x8 )
                            x264_me_refine_qpel_rd( h, &analysis.l0.me8x8[i], analysis.i_lambda2, i*4, 0 );
                        else if( h->mb.i_sub_partition[i] == D_L1_8x8 )
                            x264_me_refine_qpel_rd( h, &analysis.l1.me8x8[i], analysis.i_lambda2, i*4, 1 );
                        else if( h->mb.i_sub_partition[i] == D_BI_8x8 )
                        {
                            i_biweight = h->mb.bipred_weight[analysis.l0.me8x8[i].i_ref][analysis.l1.me8x8[i].i_ref];
                            x264_me_refine_bidir_rd( h, &analysis.l0.me8x8[i], &analysis.l1.me8x8[i], i_biweight, i, analysis.i_lambda2 );
                        }
                    }
                }
            }
        }
    }
//} macroblock_analyse_B //} macroblock_analyse_B //} macroblock_analyse_B //} macroblock_analyse_B

    dull_analyse_update_cache( h, &analysis );

    /* In rare cases we can end up qpel-RDing our way back to a larger partition size
     * without realizing it.  Check for this and account for it if necessary. */
    if( analysis.i_mbrd >= 2 )
    {
        /* Don't bother with bipred or 8x8-and-below, the odds are incredibly low. */
        static const uint8_t check_mv_lists[X264_MBTYPE_MAX] = {0,0,0,0,1,0,0,0,1,0,0,0,2};//{[P_L0]=1, [B_L0_L0]=1, [B_L1_L1]=2};
        int list = check_mv_lists[h->mb.i_type] - 1;
        if( list >= 0 && h->mb.i_partition != D_16x16 &&
            M32( &h->mb.cache.mv[list][x264_scan8[0]] ) == M32( &h->mb.cache.mv[list][x264_scan8[12]] ) &&
            h->mb.cache.ref[list][x264_scan8[0]] == h->mb.cache.ref[list][x264_scan8[12]] )
                h->mb.i_partition = D_16x16;
    }

    if( !analysis.i_mbrd )
        x264_mb_analyse_transform( h );

    if( analysis.i_mbrd == 3 && !IS_SKIP(h->mb.i_type) )
        x264_mb_analyse_qp_rd( h, &analysis );

    h->mb.b_trellis = h->param.analyse.i_trellis;
    h->mb.b_noise_reduction = !!h->param.analyse.i_noise_reduction;
    if( !IS_SKIP(h->mb.i_type) && h->mb.i_psy_trellis && h->param.analyse.i_trellis == 1 )
        x264_psy_trellis_init( h, 0 );
    if( h->mb.b_trellis == 1 || h->mb.b_noise_reduction )
        h->mb.i_skip_intra = 0;
}
//} macroblock_analyse //} macroblock_analyse //} macroblock_analyse //} macroblock_analyse 

/*-------------------- Update MB from the analysis ----------------------*/
static void dull_analyse_update_cache( x264_t *h, x264_mb_analysis_t *a )
{
    int i;
    switch( h->mb.i_type )
    {
        case I_4x4:
            for( i = 0; i < 16; i++ )
                h->mb.cache.intra4x4_pred_mode[x264_scan8[i]] = a->i_predict4x4[i];

            x264_mb_analyse_intra_chroma( h, a );
            break;
        case I_8x8:
            for( i = 0; i < 4; i++ )
                x264_macroblock_cache_intra8x8_pred( h, 2*(i&1), 2*(i>>1), a->i_predict8x8[i] );

            x264_mb_analyse_intra_chroma( h, a );
            break;
        case I_16x16:
            h->mb.i_intra16x16_pred_mode = a->i_predict16x16;
            x264_mb_analyse_intra_chroma( h, a );
            break;

        case I_PCM:
            break;

        case P_L0:
            switch( h->mb.i_partition )
            {
                case D_16x16:
                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, a->l0.me16x16.i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 0, a->l0.me16x16.mv );
                    break;

                case D_16x8:
                    x264_macroblock_cache_ref( h, 0, 0, 4, 2, 0, a->l0.me16x8[0].i_ref );
                    x264_macroblock_cache_ref( h, 0, 2, 4, 2, 0, a->l0.me16x8[1].i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 2, 0, a->l0.me16x8[0].mv );
                    x264_macroblock_cache_mv_ptr( h, 0, 2, 4, 2, 0, a->l0.me16x8[1].mv );
                    break;

                case D_8x16:
                    x264_macroblock_cache_ref( h, 0, 0, 2, 4, 0, a->l0.me8x16[0].i_ref );
                    x264_macroblock_cache_ref( h, 2, 0, 2, 4, 0, a->l0.me8x16[1].i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 2, 4, 0, a->l0.me8x16[0].mv );
                    x264_macroblock_cache_mv_ptr( h, 2, 0, 2, 4, 0, a->l0.me8x16[1].mv );
                    break;

                default:
                    x264_log( h, X264_LOG_ERROR, "internal error P_L0 and partition=%d\n", h->mb.i_partition );
                    break;
            }
            break;

        case P_8x8:
            x264_macroblock_cache_ref( h, 0, 0, 2, 2, 0, a->l0.me8x8[0].i_ref );
            x264_macroblock_cache_ref( h, 2, 0, 2, 2, 0, a->l0.me8x8[1].i_ref );
            x264_macroblock_cache_ref( h, 0, 2, 2, 2, 0, a->l0.me8x8[2].i_ref );
            x264_macroblock_cache_ref( h, 2, 2, 2, 2, 0, a->l0.me8x8[3].i_ref );
            for( i = 0; i < 4; i++ )
                x264_mb_cache_mv_p8x8( h, a, i );
            break;

        case P_SKIP:
        {
            h->mb.i_partition = D_16x16;
            x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, 0 );
            x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 0, h->mb.cache.pskip_mv );
            break;
        }

        case B_SKIP:
        case B_DIRECT:
            h->mb.i_partition = h->mb.cache.direct_partition;
            x264_mb_load_mv_direct8x8( h, 0 );
            x264_mb_load_mv_direct8x8( h, 1 );
            x264_mb_load_mv_direct8x8( h, 2 );
            x264_mb_load_mv_direct8x8( h, 3 );
            break;

        case B_8x8:
            /* optimize: cache might not need to be rewritten */
            for( i = 0; i < 4; i++ )
                x264_mb_cache_mv_b8x8( h, a, i, 1 );
            break;

        default: /* the rest of the B types */
            switch( h->mb.i_partition )
            {
            case D_16x16:
                switch( h->mb.i_type )
                {
                case B_L0_L0:
                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, a->l0.me16x16.i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 0, a->l0.me16x16.mv );

                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 1, -1 );
                    x264_macroblock_cache_mv ( h, 0, 0, 4, 4, 1, 0 );
                    x264_macroblock_cache_mvd( h, 0, 0, 4, 4, 1, 0 );
                    break;
                case B_L1_L1:
                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, -1 );
                    x264_macroblock_cache_mv ( h, 0, 0, 4, 4, 0, 0 );
                    x264_macroblock_cache_mvd( h, 0, 0, 4, 4, 0, 0 );

                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 1, a->l1.me16x16.i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 1, a->l1.me16x16.mv );
                    break;
                case B_BI_BI:
                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 0, a->l0.bi16x16.i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 0, a->l0.bi16x16.mv );

                    x264_macroblock_cache_ref( h, 0, 0, 4, 4, 1, a->l1.bi16x16.i_ref );
                    x264_macroblock_cache_mv_ptr( h, 0, 0, 4, 4, 1, a->l1.bi16x16.mv );
                    break;
                }
                break;
            case D_16x8:
                x264_mb_cache_mv_b16x8( h, a, 0, 1 );
                x264_mb_cache_mv_b16x8( h, a, 1, 1 );
                break;
            case D_8x16:
                x264_mb_cache_mv_b8x16( h, a, 0, 1 );
                x264_mb_cache_mv_b8x16( h, a, 1, 1 );
                break;
            default:
                x264_log( h, X264_LOG_ERROR, "internal error (invalid MB type)\n" );
                break;
            }
    }
}

#endif // _MCDULL_ANALYSE_
