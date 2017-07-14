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
#ifdef _MCDULL_ME_0_

#include "mcdull_core.h"

//{ me_search_ref //{ me_search_ref //{ me_search_ref //{ me_search_ref 
void dull_me_search_ref_0( x264_t *h, x264_me_t *m, int16_t (*mvc)[2], int i_mvc, int *p_halfpel_thresh )
{
    const uint16_t *p_cost_mvx = m->p_cost_mv - m->mvp[0];
    const uint16_t *p_cost_mvy = m->p_cost_mv - m->mvp[1];

    m->mv[0] = 0;
    m->mv[1] = 0;
    /* compute the real cost */
    m->cost_mv = p_cost_mvx[ m->mv[0] ] + p_cost_mvy[ m->mv[1] ];
}
//} me_search_ref //} me_search_ref //} me_search_ref //} me_search_ref 

#endif // _MCDULL_ME_0_
