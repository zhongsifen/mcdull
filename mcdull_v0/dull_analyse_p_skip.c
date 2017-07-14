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

#ifdef _MCDULL_ANALYSE_P_SKIP_

#include "mcdull_core.h"

//{ mb_analyse_int //{ mb_analyse_int //{ mb_analyse_int //{ mb_analyse_int 
static void dull_mb_analyse_init_P_SKIP( x264_t *h, x264_mb_analysis_t *a )
{
    x264_mb_analyse_init_qp( h, a, h->mb.i_qp );

    /* II: Inter part P/B frame */
    if( h->sh.i_type != SLICE_TYPE_I )
    {
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

        h->mb.b_skip_mc = 0;

            a->b_force_intra = 0;
    }
}
//} mb_analyse_int //} mb_analyse_int //} mb_analyse_int //} mb_analyse_int 


//{ macroblock_analyse //{ macroblock_analyse //{ macroblock_analyse //{ macroblock_analyse 
/*****************************************************************************
 * x264_macroblock_analyse:
 *****************************************************************************/
void dull_macroblock_analyse_P_SKIP( x264_t *h )
{
    x264_mb_analysis_t analysis;
    int i;

    dull_mb_analyse_init_P( h, &analysis );

    /*--------------------------- Do the analysis ---------------------------*/
    assert ( h->sh.i_type == SLICE_TYPE_P );
//{ macroblock_analyse_P //{ macroblock_analyse_P //{ macroblock_analyse_P //{ macroblock_analyse_P 
    {
        analysis.b_try_skip = 0;

        {
            x264_mb_analyse_load_costs( h, &analysis );

            h->mb.i_type = P_SKIP;

            assert( h->mb.i_type == P_SKIP );
            {
                for( i = 1; i < h->mb.pic.i_fref[0]; i++ )
                    M32( h->mb.mvr[0][i][h->mb.i_mb_xy] ) = 0;
            }
        }
    }
//} macroblock_analyse_P //} macroblock_analyse_P //} macroblock_analyse_P //} macroblock_analyse_P 

    dull_analyse_update_cache_P( h, &analysis );
}
//} macroblock_analyse //} macroblock_analyse //} macroblock_analyse //} macroblock_analyse 

#endif // _MCDULL_ANALYSE_P_SKIP_
