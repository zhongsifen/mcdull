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
#ifdef _MCDULL_ME_

#include "mcdull_core.h"

//{ me_search_ref //{ me_search_ref //{ me_search_ref //{ me_search_ref 
void dull_me_search_ref( x264_t *h, x264_me_t *m, int16_t (*mvc)[2], int i_mvc, int *p_halfpel_thresh )
{
    const int bw = x264_pixel_size[m->i_pixel].w;
    const int bh = x264_pixel_size[m->i_pixel].h;
    const int i_pixel = m->i_pixel;
    const int stride = m->i_stride[0];
    int i_me_range = h->param.analyse.i_me_range;
    int bmx, bmy, bcost;
    int bpred_mx = 0, bpred_my = 0, bpred_cost = COST_MAX;
    int pmx, pmy;
    pixel *p_fenc = m->p_fenc[0];
    pixel *p_fref_w = m->p_fref_w;
    ALIGNED_ARRAY_16( pixel, pix,[16*16] );

    int i;
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

    uint32_t pmv;
    bmx = x264_clip3( m->mvp[0], mv_x_min_qpel, mv_x_max_qpel );
    bmy = x264_clip3( m->mvp[1], mv_y_min_qpel, mv_y_max_qpel );
    pmx = ( bmx + 2 ) >> 2;
    pmy = ( bmy + 2 ) >> 2;
    bcost = COST_MAX;

    /* try extra predictors if provided */
    if( h->mb.i_subpel_refine >= 3 )
    {
        pmv = pack16to32_mask(bmx,bmy);
        if( i_mvc )
            COST_MV_HPEL( bmx, bmy );
        for( i = 0; i < i_mvc; i++ )
        {
            if( M32( mvc[i] ) && (pmv != M32( mvc[i] )) )
            {
                int mx = x264_clip3( mvc[i][0], mv_x_min_qpel, mv_x_max_qpel );
                int my = x264_clip3( mvc[i][1], mv_y_min_qpel, mv_y_max_qpel );
                COST_MV_HPEL( mx, my );
            }
        }
        bmx = ( bpred_mx + 2 ) >> 2;
        bmy = ( bpred_my + 2 ) >> 2;
        COST_MV( bmx, bmy );
    }
    else
    {
        /* check the MVP */
        bmx = pmx;
        bmy = pmy;
        /* Because we are rounding the predicted motion vector to fullpel, there will be
         * an extra MV cost in 15 out of 16 cases.  However, when the predicted MV is
         * chosen as the best predictor, it is often the case that the subpel search will
         * result in a vector at or next to the predicted motion vector.  Therefore, it is
         * sensible to omit the cost of the MV from the rounded MVP to avoid unfairly
         * biasing against use of the predicted motion vector. */
        bcost = h->pixf.fpelcmp[i_pixel]( p_fenc, FENC_STRIDE, &p_fref_w[bmy*stride+bmx], stride );
        pmv = pack16to32_mask( bmx, bmy );
        if( i_mvc > 0 )
        {
            int i;
            ALIGNED_ARRAY_8( int16_t, mvc_fpel,[16],[2] );
            x264_predictor_roundclip( mvc_fpel, mvc, i_mvc, mv_x_min, mv_x_max, mv_y_min, mv_y_max );
            bcost <<= 4;
            for( i = 1; i <= i_mvc; i++ )
            {
                if( M32( mvc_fpel[i-1] ) && (pmv != M32( mvc[i-1] )) )
                {
                    int mx = mvc_fpel[i-1][0];
                    int my = mvc_fpel[i-1][1];
                    int cost = h->pixf.fpelcmp[i_pixel]( p_fenc, FENC_STRIDE, &p_fref_w[my*stride+mx], stride ) + BITS_MVD( mx, my );
                    cost = (cost << 4) + i;
                    COPY1_IF_LT( bcost, cost );
                }
            }
            if( bcost&15 )
            {
                bmx = mvc_fpel[(bcost&15)-1][0];
                bmy = mvc_fpel[(bcost&15)-1][1];
            }
            bcost >>= 4;
        }
    }

    if( pmv )
        COST_MV( 0, 0 );

    switch( h->mb.i_me_method )
    {
        case X264_ME_DIA:
        {
            /* diamond search, radius 1 */
            int i = i_me_range;
            bcost <<= 4;
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
            } while( --i && CHECK_MVRANGE(bmx, bmy) );
            bcost >>= 4;
            break;
        }

        case X264_ME_HEX:
        {
            int dir;
    //me_hex2:
            /* hexagon search, radius 2 */
    #if 0
            for( int i = 0; i < i_me_range/2; i++ )
            {
                omx = bmx; omy = bmy;
                COST_MV( omx-2, omy   );
                COST_MV( omx-1, omy+2 );
                COST_MV( omx+1, omy+2 );
                COST_MV( omx+2, omy   );
                COST_MV( omx+1, omy-2 );
                COST_MV( omx-1, omy-2 );
                if( bmx == omx && bmy == omy )
                    break;
                if( !CHECK_MVRANGE(bmx, bmy) )
                    break;
            }
    #else
            /* equivalent to the above, but eliminates duplicate candidates */

            /* hexagon */
            COST_MV_X3_DIR( -2,0, -1, 2,  1, 2, costs   );
            COST_MV_X3_DIR(  2,0,  1,-2, -1,-2, costs+3 );
            bcost <<= 3;
            COPY1_IF_LT( bcost, (costs[0]<<3)+2 );
            COPY1_IF_LT( bcost, (costs[1]<<3)+3 );
            COPY1_IF_LT( bcost, (costs[2]<<3)+4 );
            COPY1_IF_LT( bcost, (costs[3]<<3)+5 );
            COPY1_IF_LT( bcost, (costs[4]<<3)+6 );
            COPY1_IF_LT( bcost, (costs[5]<<3)+7 );

            if( bcost&7 )
            {
                int i;
                dir = (bcost&7)-2;
                bmx += hex2[dir+1][0];
                bmy += hex2[dir+1][1];

                /* half hexagon, not overlapping the previous iteration */
                for( i = (i_me_range>>1) - 1; i > 0 && CHECK_MVRANGE(bmx, bmy); i-- )
                {
                    COST_MV_X3_DIR( hex2[dir+0][0], hex2[dir+0][1],
                                    hex2[dir+1][0], hex2[dir+1][1],
                                    hex2[dir+2][0], hex2[dir+2][1],
                                    costs );
                    bcost &= ~7;
                    COPY1_IF_LT( bcost, (costs[0]<<3)+1 );
                    COPY1_IF_LT( bcost, (costs[1]<<3)+2 );
                    COPY1_IF_LT( bcost, (costs[2]<<3)+3 );
                    if( !(bcost&7) )
                        break;
                    dir += (bcost&7)-2;
                    dir = mod6m1[dir+1];
                    bmx += hex2[dir+1][0];
                    bmy += hex2[dir+1][1];
                }
            }
            bcost >>= 3;
    #endif
            /* square refine */
            dir = 0;
            COST_MV_X4_DIR(  0,-1,  0,1, -1,0, 1,0, costs );
            COPY2_IF_LT( bcost, costs[0], dir, 1 );
            COPY2_IF_LT( bcost, costs[1], dir, 2 );
            COPY2_IF_LT( bcost, costs[2], dir, 3 );
            COPY2_IF_LT( bcost, costs[3], dir, 4 );
            COST_MV_X4_DIR( -1,-1, -1,1, 1,-1, 1,1, costs );
            COPY2_IF_LT( bcost, costs[0], dir, 5 );
            COPY2_IF_LT( bcost, costs[1], dir, 6 );
            COPY2_IF_LT( bcost, costs[2], dir, 7 );
            COPY2_IF_LT( bcost, costs[3], dir, 8 );
            bmx += square1[dir][0];
            bmy += square1[dir][1];
            break;
        }
    }

    /* -> qpel mv */
    if( bpred_cost < bcost )
    {
        m->mv[0] = bpred_mx;
        m->mv[1] = bpred_my;
        m->cost = bpred_cost;
    }
    else
    {
        m->mv[0] = bmx << 2;
        m->mv[1] = bmy << 2;
        m->cost = bcost;
    }

    /* compute the real cost */
    m->cost_mv = p_cost_mvx[ m->mv[0] ] + p_cost_mvy[ m->mv[1] ];
    if( bmx == pmx && bmy == pmy && h->mb.i_subpel_refine < 3 )
        m->cost += m->cost_mv;

    /* subpel refine */
    if( h->mb.i_subpel_refine >= 2 )
    {
        int hpel = subpel_iterations[h->mb.i_subpel_refine][2];
        int qpel = subpel_iterations[h->mb.i_subpel_refine][3];
        refine_subpel( h, m, hpel, qpel, p_halfpel_thresh, 0 );
    }
}
//} me_search_ref //} me_search_ref //} me_search_ref //} me_search_ref 

#endif // _MCDULL_ME_
