/*****************************************************************************
 * me.c: motion estimation
 *****************************************************************************
 * Copyright (C) 2003-2010 x264 project
 *
 * Authors: Loren Merritt <lorenm@u.washington.edu>
 *          Laurent Aimar <fenrir@via.ecp.fr>
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
#ifdef _MCDULL_ME_1_

#include "mcdull_core.h"

//{ me_search_ref //{ me_search_ref //{ me_search_ref //{ me_search_ref 
void dull_me_search_ref_1( x264_t *h, x264_me_t *m, int16_t (*mvc)[2], int i_mvc, int *p_halfpel_thresh )
{
    const int bw = x264_pixel_size[m->i_pixel].w;
    const int bh = x264_pixel_size[m->i_pixel].h;
    const int i_pixel = m->i_pixel;
    const int stride = m->i_stride[0];
    int i_me_range = h->param.analyse.i_me_range;
    int bmx, bmy, bcost;
    int pmx, pmy;
    pixel *p_fenc = m->p_fenc[0];
    pixel *p_fref_w = m->p_fref_w;

    int costs[16];

    int mv_x_min = h->mb.mv_min_fpel[0];
    int mv_y_min = h->mb.mv_min_fpel[1];
    int mv_x_max = h->mb.mv_max_fpel[0];
    int mv_y_max = h->mb.mv_max_fpel[1];
    int mv_x_min_qpel = mv_x_min << 2;
    int mv_y_min_qpel = mv_y_min << 2;
    int mv_x_max_qpel = mv_x_max << 2;
    int mv_y_max_qpel = mv_y_max << 2;
/* Special version of pack to allow shortcuts in CHECK_MVRANGE */
#define pack16to32_mask2(mx,my) ((mx<<16)|(my&0x7FFF))
    uint32_t mv_min = pack16to32_mask2( -mv_x_min, -mv_y_min );
    uint32_t mv_max = pack16to32_mask2( mv_x_max, mv_y_max )|0x8000;

#define CHECK_MVRANGE(mx,my) (!(((pack16to32_mask2(mx,my) + mv_min) | (mv_max - pack16to32_mask2(mx,my))) & 0x80004000))

    const uint16_t *p_cost_mvx = m->p_cost_mv - m->mvp[0];
    const uint16_t *p_cost_mvy = m->p_cost_mv - m->mvp[1];

    bmx = x264_clip3( m->mvp[0], mv_x_min_qpel, mv_x_max_qpel );
    bmy = x264_clip3( m->mvp[1], mv_y_min_qpel, mv_y_max_qpel );
    pmx = ( bmx + 2 ) >> 2;
    pmy = ( bmy + 2 ) >> 2;
    bcost = COST_MAX;

            /* diamond search, radius 1 */
            do
            {
                COST_MV_X4_DIR( 0,-1, 0,1, -1,0, 1,0, costs );
                COPY1_IF_LT( bcost, (costs[0]<<4)+ 1 );
                COPY1_IF_LT( bcost, (costs[1]<<4)+ 3 );
                COPY1_IF_LT( bcost, (costs[2]<<4)+ 4 );
                COPY1_IF_LT( bcost, (costs[3]<<4)+12 );
                if( !(bcost&15) )
                    break;
                bmx -= (bcost<<28)>>30;
                bmy -= (bcost<<30)>>30;
                bcost &= ~15;
            } while( 0 );
            bcost >>= 4;

    /* -> qpel mv */
    {
        m->mv[0] = bmx << 2;
        m->mv[1] = bmy << 2;
        m->cost = bcost;
    }

    /* compute the real cost */
    m->cost_mv = p_cost_mvx[ m->mv[0] ] + p_cost_mvy[ m->mv[1] ];

    /* subpel refine */
    if( h->mb.i_subpel_refine >= 2 )
    {
        int hpel = subpel_iterations[h->mb.i_subpel_refine][2];
        int qpel = subpel_iterations[h->mb.i_subpel_refine][3];
        refine_subpel( h, m, hpel, qpel, p_halfpel_thresh, 0 );
    }
}
//} me_search_ref //} me_search_ref //} me_search_ref //} me_search_ref 

#endif // _MCDULL_ME_1_
