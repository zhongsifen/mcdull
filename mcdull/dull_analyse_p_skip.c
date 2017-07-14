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
            h->mb.i_type = P_SKIP;
            h->mb.i_partition = D_16x16;
            assert( h->mb.cache.pskip_mv[1] <= h->mb.mv_max_spel[1] || h->i_thread_frames == 1 );
            /* Set up MVs for future predictors */
            for( i = 0; i < h->mb.pic.i_fref[0]; i++ )
                M32( h->mb.mvr[0][i][h->mb.i_mb_xy] ) = 0;
        }
//} macroblock_analyse_P //} macroblock_analyse_P //} macroblock_analyse_P //} macroblock_analyse_P 

    dull_analyse_update_cache_P( h, &analysis );
}
//} macroblock_analyse //} macroblock_analyse //} macroblock_analyse //} macroblock_analyse 

#endif // _MCDULL_ANALYSE_P_SKIP_
