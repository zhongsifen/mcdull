/*****************************************************************************
 * cpu.h: cpu detection
 *****************************************************************************
 * Copyright (C) 2004-2010 x264 project
 *
 * Authors: Loren Merritt <lorenm@u.washington.edu>
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

#ifndef X264_CPU_H
#define X264_CPU_H

uint32_t x264_cpu_detect( void );
int      x264_cpu_num_processors( void );
int      x264_cpu_num_physical_cores( void );
void     x264_cpu_sfence( void );
#define x264_emms()
#define x264_sfence x264_cpu_sfence
void     x264_cpu_mask_misalign_sse( void );

typedef struct {
    const char name[16];
    int flags;
} x264_cpu_name_t;
extern const x264_cpu_name_t x264_cpu_names[];

#endif
