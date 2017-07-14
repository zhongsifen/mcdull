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
#ifdef _MCDULL_ANALYSE_P_INTRA_

#include "mcdull_core.h"

static void dull_analyse_update_cache_P( x264_t *h, x264_mb_analysis_t *a );


//{ macroblock_analyse //{ macroblock_analyse //{ macroblock_analyse //{ macroblock_analyse 
/*****************************************************************************
 * x264_macroblock_analyse:
 *****************************************************************************/
void dull_macroblock_analyse_P_intra( x264_t *h )
{
    x264_mb_analysis_t analysis;
    int i_cost = COST_MAX;

    dull_mb_analyse_init_P( h, &analysis );

    /*--------------------------- Do the analysis ---------------------------*/
    assert ( h->sh.i_type == SLICE_TYPE_P );
//{ macroblock_analyse_P //{ macroblock_analyse_P //{ macroblock_analyse_P //{ macroblock_analyse_P 
    {
        int b_skip = 0;

        h->mc.prefetch_ref( h->mb.pic.p_fref[0][0][h->mb.i_mb_x&3], h->mb.pic.i_stride[0], 0 );
		{
            int i_type;
                x264_mb_analyse_intra( h, &analysis, i_cost );

            COPY2_IF_LT( i_cost, analysis.i_satd_i16x16, i_type, I_16x16 );

            h->mb.i_type = i_type;
		}
    }
//} macroblock_analyse_P //} macroblock_analyse_P //} macroblock_analyse_P //} macroblock_analyse_P 
    assert ( h->sh.i_type != SLICE_TYPE_B );
//{ macroblock_analyse_B //{ macroblock_analyse_B //{ macroblock_analyse_B //{ macroblock_analyse_B
//} macroblock_analyse_B //} macroblock_analyse_B //} macroblock_analyse_B //} macroblock_analyse_B

    x264_analyse_update_cache( h, &analysis );
}
//} macroblock_analyse //} macroblock_analyse //} macroblock_analyse //} macroblock_analyse 

#endif // _MCDULL_ANALYSE_P_intra_
