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

#ifdef _MCDULL_ANALYSE_P_GOOD_

#include "mcdull_core.h"

static void dull_analyse_update_cache_P( x264_t *h, x264_mb_analysis_t *a );

//{ mb_analyse_inter_p16x16 //{ mb_analyse_inter_p16x16 //{ mb_analyse_inter_p16x16 //{ mb_analyse_inter_p16x16
static void dull_mb_analyse_inter_p16x16_1( x264_t *h, x264_mb_analysis_t *a )
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

        {
            x264_mb_predict_mv_ref16x16( h, 0, i_ref, mvc, &i_mvc );
            dull_me_search_ref_1( h, &m, mvc, i_mvc, NULL );
        }

        /* save mv for predicting neighbors */
        CP32( h->mb.mvr[0][i_ref][h->mb.i_mb_xy], m.mv );
        CP32( a->l0.mvc[i_ref][0], m.mv );

        /* early termination
         * SSD threshold would probably be better than SATD */
        if( i_ref == 0
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
static void dull_mb_analyse_inter_p8x8_1( x264_t *h, x264_mb_analysis_t *a )
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
        dull_me_search_ref_1( h, m, mvc, i_mvc, NULL );

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

//{ macroblock_analyse //{ macroblock_analyse //{ macroblock_analyse //{ macroblock_analyse 
/*****************************************************************************
 * x264_macroblock_analyse:
 *****************************************************************************/
void dull_macroblock_analyse_P_GOOD( x264_t *h )
{
    x264_mb_analysis_t analysis;
    int i_cost = COST_MAX;
    int i;

    dull_mb_analyse_init_P( h, &analysis );

    /*--------------------------- Do the analysis ---------------------------*/
    assert ( h->sh.i_type == SLICE_TYPE_P );
//{ macroblock_analyse_P //{ macroblock_analyse_P //{ macroblock_analyse_P //{ macroblock_analyse_P 
    {
        analysis.b_try_skip = 0;
        assert( !analysis.b_force_intra );
        {
            const unsigned int flags = h->param.analyse.inter;
            int i_type;
            int i_partition;

            x264_mb_analyse_load_costs( h, &analysis );

            dull_mb_analyse_inter_p16x16_1( h, &analysis );

            if( h->mb.i_type == P_SKIP )
            {
                for( i = 1; i < h->mb.pic.i_fref[0]; i++ )
                    M32( h->mb.mvr[0][i][h->mb.i_mb_xy] ) = 0;
                return;
            }

            /* Select best inter mode */
            i_type = P_L0;
            i_partition = D_16x16;
            i_cost = analysis.l0.me16x16.cost;
            h->mb.i_partition = i_partition;
            h->mb.i_type = i_type;
        }
    }
//} macroblock_analyse_P //} macroblock_analyse_P //} macroblock_analyse_P //} macroblock_analyse_P 

    dull_analyse_update_cache_P( h, &analysis );
}
//} macroblock_analyse //} macroblock_analyse //} macroblock_analyse //} macroblock_analyse 

#endif // _MCDULL_ANALYSE_P_GOOD_
