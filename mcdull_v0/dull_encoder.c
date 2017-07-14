/*****************************************************************************
 * encoder.c: top-level encoder functions
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

#ifdef _MCDULL_ENCODER_

#include "mcdull_core.h"

/****************************************************************************
 *
 ******************************* x264 libs **********************************
 *
 ****************************************************************************/
/****************************************************************************
 *
 ****************************************************************************
 ****************************** External API*********************************
 ****************************************************************************
 *
 ****************************************************************************/
/****************************************************************************
 * x264_encoder_open:
 ****************************************************************************/
/****************************************************************************
 * x264_encoder_reconfig:
 ****************************************************************************/
/****************************************************************************
 * x264_encoder_parameters:
 ****************************************************************************/
/****************************************************************************
 * x264_encoder_headers:
 ****************************************************************************/
int dull_encoder_headers( x264_t *h, x264_nal_t **pp_nal, int *pi_nal )
{
    int frame_size = 0;
    /* init bitstream context */
    h->out.i_nal = 0;
    bs_init( &h->out.bs, h->out.p_bitstream, h->out.i_bitstream );

    /* Write SEI, SPS and PPS. */

    /* generate sequence parameters */
    x264_nal_start( h, NAL_SPS, NAL_PRIORITY_HIGHEST );
    x264_sps_write( &h->out.bs, h->sps );
    if( x264_nal_end( h ) )
        return -1;

    /* generate picture parameters */
    x264_nal_start( h, NAL_PPS, NAL_PRIORITY_HIGHEST );
    x264_pps_write( &h->out.bs, h->pps );
    if( x264_nal_end( h ) )
        return -1;

    /* identify ourselves */
    if (h->param.b_version_info)
    {
        x264_nal_start( h, NAL_SEI, NAL_PRIORITY_DISPOSABLE );
        if( x264_sei_version_write( h, &h->out.bs ) )
            return -1;
        if( x264_nal_end( h ) )
            return -1;
    }

    frame_size = x264_encoder_encapsulate_nals( h, 0 );

    /* now set output*/
    *pi_nal = h->out.i_nal;
    *pp_nal = &h->out.nal[0];
    h->out.i_nal = 0;

    return frame_size;
}

//{ _slice_write //{ _slice_write //{ _slice_write //{ _slice_write //{ _slice_write //{ _slice_write //{ _slice_write //{ _slice_write 
static int dull_slice_write( McDull_t* McDull )
{
    x264_t *h = McDull->x264;

    int i_skip;
    int mb_xy, i_mb_x, i_mb_y;
    int i_skip_bak = 0; /* Shut up GCC. */
    bs_t bs_bak;
    x264_cabac_t cabac_bak;
    uint8_t cabac_prevbyte_bak = 0; /* Shut up GCC. */
    int mv_bits_bak = 0;
    int tex_bits_bak = 0;
    /* NALUs other than the first use a 3-byte startcode.
     * Add one extra byte for the rbsp, and one more for the final CABAC putbyte.
     * Then add an extra 5 bytes just in case, to account for random NAL escapes and
     * other inaccuracies. */
    int overhead_guess = (NALU_OVERHEAD - (h->param.b_annexb && h->out.i_nal)) + 1 + h->param.b_cabac + 5;
    int slice_max_size = h->param.i_slice_max_size > 0 ? (h->param.i_slice_max_size-overhead_guess)*8 : 0;
    int starting_bits = bs_pos(&h->out.bs);
    int b_deblock = h->sh.i_disable_deblocking_filter_idc != 1;
    int b_hpel = h->fdec->b_kept_as_ref;
    b_deblock &= b_hpel || h->param.psz_dump_yuv;
    bs_realign( &h->out.bs );

    /* Slice */
    x264_nal_start( h, h->i_nal_type, h->i_nal_ref_idc );
    h->out.nal[h->out.i_nal].i_first_mb = h->sh.i_first_mb;

    /* Slice header */
    x264_macroblock_thread_init( h );

    /* If this isn't the first slice in the threadslice, set the slice QP
     * equal to the last QP in the previous slice for more accurate
     * CABAC initialization. */
    if( h->sh.i_first_mb != h->i_threadslice_start * h->mb.i_mb_width )
    {
        h->sh.i_qp = h->mb.i_last_qp;
        h->sh.i_qp_delta = h->sh.i_qp - h->pps->i_pic_init_qp;
    }

    x264_slice_header_write( &h->out.bs, &h->sh, h->i_nal_ref_idc );
    if( h->param.b_cabac )
    {
        /* alignment needed */
        bs_align_1( &h->out.bs );

        /* init cabac */
        x264_cabac_context_init( &h->cabac, h->sh.i_type, x264_clip3( h->sh.i_qp-QP_BD_OFFSET, 0, 51 ), h->sh.i_cabac_init_idc );
        x264_cabac_encode_init ( &h->cabac, h->out.bs.p, h->out.bs.p_end );
    }
    h->mb.i_last_qp = h->sh.i_qp;
    h->mb.i_last_dqp = 0;

    i_mb_y = h->sh.i_first_mb / h->mb.i_mb_width;
    i_mb_x = h->sh.i_first_mb % h->mb.i_mb_width;
    i_skip = 0;

    while( (mb_xy = i_mb_x + i_mb_y * h->mb.i_mb_width) <= h->sh.i_last_mb )
    {
        int total_bits, mb_size, b_intra;
        int mb_spos = bs_pos(&h->out.bs) + x264_cabac_pos(&h->cabac);

        if( x264_bitstream_check_buffer( h ) )
            return -1;

        if( slice_max_size )
        {
            mv_bits_bak = h->stat.frame.i_mv_bits;
            tex_bits_bak = h->stat.frame.i_tex_bits;
            /* We don't need the contexts because flushing the CABAC encoder has no context
             * dependency and macroblocks are only re-encoded in the case where a slice is
             * ended (and thus the content of all contexts are thrown away). */
            if( h->param.b_cabac )
            {
                memcpy( &cabac_bak, &h->cabac, offsetof(x264_cabac_t, f8_bits_encoded) );
                /* x264's CABAC writer modifies the previous byte during carry, so it has to be
                 * backed up. */
                cabac_prevbyte_bak = h->cabac.p[-1];
            }
            else
            {
                bs_bak = h->out.bs;
                i_skip_bak = i_skip;
            }
        }

        if( i_mb_x == 0 && !h->mb.b_reencode_mb )
            x264_fdec_filter_row( h, i_mb_y, 1 );

        /* load cache */
        x264_macroblock_cache_load( h, i_mb_x, i_mb_y );

#if mcdull_SWATAW
        McDull_swataw( McDull );
#else
        x264_macroblock_analyse( h );

        /* encode this macroblock -> be careful it can change the mb type to P_SKIP if needed */
        x264_macroblock_encode( h );
#endif

        if( h->param.b_cabac )
        {
            if( mb_xy > h->sh.i_first_mb )
                x264_cabac_encode_terminal( &h->cabac );

            if( IS_SKIP( h->mb.i_type ) )
                x264_cabac_mb_skip( h, 1 );
            else
            {
                if( h->sh.i_type != SLICE_TYPE_I )
                    x264_cabac_mb_skip( h, 0 );
                x264_macroblock_write_cabac( h, &h->cabac );
            }
        }
        else
        {
            if( IS_SKIP( h->mb.i_type ) )
                i_skip++;
            else
            {
                if( h->sh.i_type != SLICE_TYPE_I )
                {
                    bs_write_ue( &h->out.bs, i_skip );  /* skip run */
                    i_skip = 0;
                }
                x264_macroblock_write_cavlc( h );
            }
        }

        total_bits = bs_pos(&h->out.bs) + x264_cabac_pos(&h->cabac);
        mb_size = total_bits - mb_spos;

        if( slice_max_size )
        {
            /* Count the skip run, just in case. */
            if( !h->param.b_cabac )
                total_bits += bs_size_ue_big( i_skip );
            /* HACK: we assume no more than 3 bytes of NALU escaping, but
             * this can fail in CABAC streams with an extremely large number of identical
             * blocks in sequence (e.g. all-black intra blocks).
             * Thus, every 64 blocks, pretend we've used a byte.
             * For reference, a seqeuence of identical empty-CBP i16x16 blocks will use
             * one byte after 26 macroblocks, assuming a perfectly adapted CABAC.
             * That's 78 macroblocks to generate the 3-byte sequence to trigger an escape. */
            else if( ((mb_xy - h->sh.i_first_mb) & 63) == 63 )
                slice_max_size -= 8;
            /* We'll just re-encode this last macroblock if we go over the max slice size. */
            if( total_bits - starting_bits > slice_max_size && !h->mb.b_reencode_mb )
            {
                if( mb_xy != h->sh.i_first_mb )
                {
                    h->stat.frame.i_mv_bits = mv_bits_bak;
                    h->stat.frame.i_tex_bits = tex_bits_bak;
                    if( h->param.b_cabac )
                    {
                        memcpy( &h->cabac, &cabac_bak, offsetof(x264_cabac_t, f8_bits_encoded) );
                        h->cabac.p[-1] = cabac_prevbyte_bak;
                    }
                    else
                    {
                        h->out.bs = bs_bak;
                        i_skip = i_skip_bak;
                    }
                    h->mb.b_reencode_mb = 1;
                    h->sh.i_last_mb = mb_xy-1;
                    break;
                }
                else
                {
                    h->sh.i_last_mb = mb_xy;
                    h->mb.b_reencode_mb = 0;
                }
            }
            else
                h->mb.b_reencode_mb = 0;
        }

        /* save cache */
        x264_macroblock_cache_save( h );

        /* accumulate mb stats */
        h->stat.frame.i_mb_count[h->mb.i_type]++;

        b_intra = IS_INTRA( h->mb.i_type );
        if( h->param.i_log_level >= X264_LOG_INFO || h->param.rc.b_stat_write )
        {
            if( !b_intra && !IS_SKIP( h->mb.i_type ) && !IS_DIRECT( h->mb.i_type ) )
            {
                int i, i_list;
                if( h->mb.i_partition != D_8x8 )
                        h->stat.frame.i_mb_partition[h->mb.i_partition] += 4;
                    else
                        for( i = 0; i < 4; i++ )
                            h->stat.frame.i_mb_partition[h->mb.i_sub_partition[i]] ++;
                if( h->param.i_frame_reference > 1 )
                    for( i_list = 0; i_list <= (h->sh.i_type == SLICE_TYPE_B); i_list++ )
                        for( i = 0; i < 4; i++ )
                        {
                            int i_ref = h->mb.cache.ref[i_list][ x264_scan8[4*i] ];
                            if( i_ref >= 0 )
                                h->stat.frame.i_mb_count_ref[i_list][i_ref] ++;
                        }
            }
        }

        if( h->param.i_log_level >= X264_LOG_INFO )
        {
            if( h->mb.i_cbp_luma | h->mb.i_cbp_chroma )
            {
                int cbpsum = (h->mb.i_cbp_luma&1) + ((h->mb.i_cbp_luma>>1)&1)
                           + ((h->mb.i_cbp_luma>>2)&1) + (h->mb.i_cbp_luma>>3);
                h->stat.frame.i_mb_cbp[!b_intra + 0] += cbpsum;
                h->stat.frame.i_mb_cbp[!b_intra + 2] += !!h->mb.i_cbp_chroma;
                h->stat.frame.i_mb_cbp[!b_intra + 4] += h->mb.i_cbp_chroma >> 1;
            }
            if( h->mb.i_cbp_luma && !b_intra )
            {
                h->stat.frame.i_mb_count_8x8dct[0] ++;
                h->stat.frame.i_mb_count_8x8dct[1] += h->mb.b_transform_8x8;
            }
            if( b_intra && h->mb.i_type != I_PCM )
            {
                int i;
                if( h->mb.i_type == I_16x16 )
                    h->stat.frame.i_mb_pred_mode[0][h->mb.i_intra16x16_pred_mode]++;
                else if( h->mb.i_type == I_8x8 )
                    for( i = 0; i < 16; i += 4 )
                        h->stat.frame.i_mb_pred_mode[1][h->mb.cache.intra4x4_pred_mode[x264_scan8[i]]]++;
                else //if( h->mb.i_type == I_4x4 )
                    for( i = 0; i < 16; i++ )
                        h->stat.frame.i_mb_pred_mode[2][h->mb.cache.intra4x4_pred_mode[x264_scan8[i]]]++;
                h->stat.frame.i_mb_pred_mode[3][x264_mb_pred_mode8x8c_fix[h->mb.i_chroma_pred_mode]]++;
            }
        }

        /* calculate deblock strength values (actual deblocking is done per-row along with hpel) */
        if( b_deblock )
        {
            int mvy_limit = 4;
            uint8_t (*bs)[4][4] = h->deblock_strength[0][h->mb.i_mb_x];
            x264_macroblock_cache_load_deblock( h );
            if( IS_INTRA( h->mb.type[h->mb.i_mb_xy] ) )
                memset( bs, 3, 2*4*4*sizeof(uint8_t) );
            else
                h->loopf.deblock_strength( h->mb.cache.non_zero_count, h->mb.cache.ref, h->mb.cache.mv,
                                           bs, mvy_limit, h->sh.i_type == SLICE_TYPE_B );
        }

        x264_ratecontrol_mb( h, mb_size );

        i_mb_x++;
        if( i_mb_x == h->mb.i_mb_width )
        {
            i_mb_y++;
            i_mb_x = 0;
        }
    }
    h->out.nal[h->out.i_nal].i_last_mb = h->sh.i_last_mb;

    if( h->param.b_cabac )
    {
        x264_cabac_encode_flush( h, &h->cabac );
        h->out.bs.p = h->cabac.p;
    }
    else
    {
        if( i_skip > 0 )
            bs_write_ue( &h->out.bs, i_skip );  /* last skip run */
        /* rbsp_slice_trailing_bits */
        bs_rbsp_trailing( &h->out.bs );
        bs_flush( &h->out.bs );
    }
    if( x264_nal_end( h ) )
        return -1;

    if( h->sh.i_last_mb == (h->i_threadslice_end * h->mb.i_mb_width - 1) )
    {
        h->stat.frame.i_misc_bits = bs_pos( &h->out.bs )
                                  + (h->out.i_nal*NALU_OVERHEAD * 8)
                                  - h->stat.frame.i_tex_bits
                                  - h->stat.frame.i_mv_bits;
        x264_fdec_filter_row( h, h->i_threadslice_end, 1 );
    }

    return 0;
}
//} _slice_write //} _slice_write //} _slice_write //} _slice_write //} _slice_write //} _slice_write //} _slice_write //} _slice_write 

//{ _slice_writ
static void *dull_slices_write( McDull_t* McDull )
{
    x264_t* h = McDull->x264;

    int i_slice_num = 0;
    int last_thread_mb = h->sh.i_last_mb;

    /* init stats */
    memset( &h->stat.frame, 0, sizeof(h->stat.frame) );
    h->mb.b_reencode_mb = 0;
    while( h->sh.i_first_mb <= last_thread_mb )
    {
        h->sh.i_last_mb = last_thread_mb;
        if( h->param.i_slice_max_mbs )
            h->sh.i_last_mb = h->sh.i_first_mb + h->param.i_slice_max_mbs - 1;
        else if( h->param.i_slice_count && !h->param.b_sliced_threads )
        {
            int height = h->mb.i_mb_height;
            int width  = h->mb.i_mb_width;
            i_slice_num++;
            h->sh.i_last_mb = (height * i_slice_num + h->param.i_slice_count/2) / h->param.i_slice_count * width - 1;
        }
        h->sh.i_last_mb = X264_MIN( h->sh.i_last_mb, last_thread_mb );
        if( dull_slice_write( McDull ) )
            return (void *)-1;
        h->sh.i_first_mb = h->sh.i_last_mb + 1;
    }

    return (void *)0;
}
//} _slice_writ

//{ _encoder_encode //{ _encoder_encode //{ _encoder_encode //{ _encoder_encode
/****************************************************************************
 * x264_encoder_encode:
 *  XXX: i_poc   : is the poc of the current given picture
 *       i_frame : is the number of the frame being coded
 *  ex:  type frame poc
 *       I      0   2*0
 *       P      1   2*3
 *       B      2   2*1
 *       B      3   2*2
 *       P      4   2*6
 *       B      5   2*4
 *       B      6   2*5
 ****************************************************************************/
int     dull_encoder_encode( McDull_t *McDull,
                             x264_nal_t **pp_nal, int *pi_nal,
                             x264_picture_t *pic_in,
                             x264_picture_t *pic_out )
{
    x264_t* h = McDull->x264;

    int i_nal_type, i_nal_ref_idc, i_global_qp;
    int overhead = NALU_OVERHEAD;

    int i, j, i_list, frame_size;
    char psz_message[80];
    int filler = 0;
    double dur;

    // ok to call this before encoding any frames, since the initial values of fdec have b_kept_as_ref=0
    if( x264_reference_update( h ) )
        return -1;
    h->fdec->i_lines_completed = -1;

    /* no data out */
    *pi_nal = 0;
    *pp_nal = NULL;

    /* ------------------- Setup new frame from picture -------------------- */
    if( pic_in != NULL )
    {
        /* 1: Copy the picture to a frame and move it to a buffer */
        x264_frame_t *fenc = x264_frame_pop_unused( h, 0 );
        if( !fenc )
            return -1;

        if( x264_frame_copy_picture( h, fenc, pic_in ) < 0 )
            return -1;

        if( h->param.i_width != 16 * h->mb.i_mb_width ||
            h->param.i_height != 16 * h->mb.i_mb_height )
            x264_frame_expand_border_mod16( h, fenc );

        fenc->i_frame = h->frames.i_input++;

        h->frames.i_second_largest_pts = h->frames.i_largest_pts;
        h->frames.i_largest_pts = fenc->i_pts;

        if( (fenc->i_pic_struct < PIC_STRUCT_AUTO) || (fenc->i_pic_struct > PIC_STRUCT_TRIPLE) )
            fenc->i_pic_struct = PIC_STRUCT_AUTO;

        if( fenc->i_pic_struct == PIC_STRUCT_AUTO )
        {
            fenc->i_pic_struct = PIC_STRUCT_PROGRESSIVE;
        }

        if( h->param.rc.b_mb_tree && h->param.rc.b_stat_read )
        {
            if( x264_macroblock_tree_read( h, fenc, pic_in->prop.quant_offsets ) )
                return -1;
        }
        else
            x264_adaptive_quant_frame( h, fenc, pic_in->prop.quant_offsets );

        if( pic_in->prop.quant_offsets_free )
            pic_in->prop.quant_offsets_free( pic_in->prop.quant_offsets );

        if( h->frames.b_have_lowres )
            x264_frame_init_lowres( h, fenc );

        /* 2: Place the frame into the queue for its slice type decision */
        x264_lookahead_put_frame( h, fenc );

        if( h->frames.i_input <= h->frames.i_delay + 1 - h->i_thread_frames )
        {
            /* Nothing yet to encode, waiting for filling of buffers */
            pic_out->i_type = X264_TYPE_AUTO;
            return 0;
        }
    }
    else
    {
        /* signal kills for lookahead thread */
        x264_pthread_mutex_lock( &h->lookahead->ifbuf.mutex );
        h->lookahead->b_exit_thread = 1;
        x264_pthread_cond_broadcast( &h->lookahead->ifbuf.cv_fill );
        x264_pthread_mutex_unlock( &h->lookahead->ifbuf.mutex );
    }

    h->i_frame++;
    /* 3: The picture is analyzed in the lookahead */
    if( !h->frames.current[0] )
        x264_lookahead_get_frames( h );

    if( !h->frames.current[0] && x264_lookahead_is_empty( h ) )
        return x264_encoder_frame_end( h, pp_nal, pi_nal, pic_out );

    /* ------------------- Get frame to be encoded ------------------------- */
    /* 4: get picture to encode */
    h->fenc = x264_frame_shift( h->frames.current );
    if( h->i_frame == h->i_thread_frames - 1 )
        h->i_reordered_pts_delay = h->fenc->i_reordered_pts;
    if( h->fenc->param )
    {
        x264_encoder_reconfig( h, h->fenc->param );
        if( h->fenc->param->param_free )
            h->fenc->param->param_free( h->fenc->param );
    }

    if( !IS_X264_TYPE_I( h->fenc->i_type ) )
    {
        int valid_refs_left = 0;
        for( i = 0; h->frames.reference[i]; i++ )
            if( !h->frames.reference[i]->b_corrupt )
                valid_refs_left++;
        /* No valid reference frames left: force an IDR. */
        if( !valid_refs_left )
        {
            h->fenc->b_keyframe = 1;
            h->fenc->i_type = X264_TYPE_IDR;
        }
    }

    if( h->fenc->b_keyframe )
    {
        h->frames.i_last_keyframe = h->fenc->i_frame;
        if( h->fenc->i_type == X264_TYPE_IDR )
        {
            h->i_frame_num = 0;
            h->frames.i_last_idr = h->fenc->i_frame;
        }
    }
    h->sh.i_mmco_command_count =
    h->sh.i_mmco_remove_from_end = 0;
    h->b_ref_reorder[0] =
    h->b_ref_reorder[1] = 0;
    h->fdec->i_poc =
    h->fenc->i_poc = 2 * ( h->fenc->i_frame - X264_MAX( h->frames.i_last_idr, 0 ) );

    /* ------------------- Setup frame context ----------------------------- */
    /* 5: Init data dependent of frame type */
    if( h->fenc->i_type == X264_TYPE_IDR )
    {
        /* reset ref pictures */
        i_nal_type    = NAL_SLICE_IDR;
        i_nal_ref_idc = NAL_PRIORITY_HIGHEST;
        h->sh.i_type = SLICE_TYPE_I;
        x264_reference_reset( h );
        h->frames.i_poc_last_open_gop = -1;
    }
    else if( h->fenc->i_type == X264_TYPE_I )
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = NAL_PRIORITY_HIGH; /* Not completely true but for now it is (as all I/P are kept as ref)*/
        h->sh.i_type = SLICE_TYPE_I;
        x264_reference_hierarchy_reset( h );
        if( h->param.i_open_gop )
            h->frames.i_poc_last_open_gop = h->fenc->b_keyframe ? h->fenc->i_poc : -1;
    }
    else if( h->fenc->i_type == X264_TYPE_P )
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = NAL_PRIORITY_HIGH; /* Not completely true but for now it is (as all I/P are kept as ref)*/
        h->sh.i_type = SLICE_TYPE_P;
        x264_reference_hierarchy_reset( h );
        h->frames.i_poc_last_open_gop = -1;
    }
    else if( h->fenc->i_type == X264_TYPE_BREF )
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = h->param.i_bframe_pyramid == X264_B_PYRAMID_STRICT ? NAL_PRIORITY_LOW : NAL_PRIORITY_HIGH;
        h->sh.i_type = SLICE_TYPE_B;
        x264_reference_hierarchy_reset( h );
    }
    else    /* B frame */
    {
        i_nal_type    = NAL_SLICE;
        i_nal_ref_idc = NAL_PRIORITY_DISPOSABLE;
        h->sh.i_type = SLICE_TYPE_B;
    }

    h->fdec->i_type = h->fenc->i_type;
    h->fdec->i_frame = h->fenc->i_frame;
    h->fenc->b_kept_as_ref =
    h->fdec->b_kept_as_ref = i_nal_ref_idc != NAL_PRIORITY_DISPOSABLE && h->param.i_keyint_max > 1;

    h->fdec->i_pts = h->fenc->i_pts;
    h->fdec->i_dts = h->fenc->i_reordered_pts;
    if( h->fenc->i_type == X264_TYPE_IDR )
        h->i_last_idr_pts = h->fdec->i_pts;

    /* ------------------- Init                ----------------------------- */
    /* build ref list 0/1 */
    x264_reference_build_list( h, h->fdec->i_poc );

    /* ---------------------- Write the bitstream -------------------------- */
    /* Init bitstream context */
    if( h->param.b_sliced_threads )
    {
        for( i = 0; i < h->param.i_threads; i++ )
        {
            bs_init( &h->thread[i]->out.bs, h->thread[i]->out.p_bitstream, h->thread[i]->out.i_bitstream );
            h->thread[i]->out.i_nal = 0;
        }
    }
    else
    {
        bs_init( &h->out.bs, h->out.p_bitstream, h->out.i_bitstream );
        h->out.i_nal = 0;
    }

    if( h->param.b_aud )
    {
        int pic_type;

        if( h->sh.i_type == SLICE_TYPE_I )
            pic_type = 0;
        else if( h->sh.i_type == SLICE_TYPE_P )
            pic_type = 1;
        else if( h->sh.i_type == SLICE_TYPE_B )
            pic_type = 2;
        else
            pic_type = 7;

        x264_nal_start( h, NAL_AUD, NAL_PRIORITY_DISPOSABLE );
        bs_write( &h->out.bs, 3, pic_type );
        bs_rbsp_trailing( &h->out.bs );
        if( x264_nal_end( h ) )
            return -1;
        overhead += h->out.nal[h->out.i_nal-1].i_payload + NALU_OVERHEAD - (h->param.b_annexb && h->out.i_nal-1);
    }

    h->i_nal_type = i_nal_type;
    h->i_nal_ref_idc = i_nal_ref_idc;

    if( h->param.b_intra_refresh )
    {
        if( IS_X264_TYPE_I( h->fenc->i_type ) )
        {
            h->fdec->i_frames_since_pir = 0;
            h->b_queued_intra_refresh = 0;
            /* PIR is currently only supported with ref == 1, so any intra frame effectively refreshes
             * the whole frame and counts as an intra refresh. */
            h->fdec->f_pir_position = h->mb.i_mb_width;
        }
        else if( h->fenc->i_type == X264_TYPE_P )
        {
            int pocdiff = (h->fdec->i_poc - h->fref0[0]->i_poc)/2;
            float increment = X264_MAX( ((float)h->mb.i_mb_width-1) / h->param.i_keyint_max, 1 );
            h->fdec->f_pir_position = h->fref0[0]->f_pir_position;
            h->fdec->i_frames_since_pir = h->fref0[0]->i_frames_since_pir + pocdiff;
            if( h->fdec->i_frames_since_pir >= h->param.i_keyint_max ||
                (h->b_queued_intra_refresh && h->fdec->f_pir_position + 0.5 >= h->mb.i_mb_width) )
            {
                h->fdec->f_pir_position = 0;
                h->fdec->i_frames_since_pir = 0;
                h->b_queued_intra_refresh = 0;
                h->fenc->b_keyframe = 1;
            }
            h->fdec->i_pir_start_col = lrintf(h->fdec->f_pir_position);
            h->fdec->f_pir_position += increment * pocdiff;
            h->fdec->i_pir_end_col = lrintf(h->fdec->f_pir_position);
            /* If our intra refresh has reached the right side of the frame, we're done. */
            if( h->fdec->i_pir_end_col >= h->mb.i_mb_width - 1 )
                h->fdec->f_pir_position = h->mb.i_mb_width;
        }
    }

    if( h->fenc->b_keyframe )
    {
        /* Write SPS and PPS */
        if( h->param.b_repeat_headers )
        {
            /* generate sequence parameters */
            x264_nal_start( h, NAL_SPS, NAL_PRIORITY_HIGHEST );
            x264_sps_write( &h->out.bs, h->sps );
            if( x264_nal_end( h ) )
                return -1;
            overhead += h->out.nal[h->out.i_nal-1].i_payload + NALU_OVERHEAD;

            /* generate picture parameters */
            x264_nal_start( h, NAL_PPS, NAL_PRIORITY_HIGHEST );
            x264_pps_write( &h->out.bs, h->pps );
            if( x264_nal_end( h ) )
                return -1;
            overhead += h->out.nal[h->out.i_nal-1].i_payload + NALU_OVERHEAD;
        }

        /* buffering period sei is written in x264_encoder_frame_end */
    }

    /* write extra sei */
    for( i = 0; i < h->fenc->extra_sei.num_payloads; i++ )
    {
        x264_nal_start( h, NAL_SEI, NAL_PRIORITY_DISPOSABLE );
        x264_sei_write( &h->out.bs, h->fenc->extra_sei.payloads[i].payload, h->fenc->extra_sei.payloads[i].payload_size,
                        h->fenc->extra_sei.payloads[i].payload_type );
        if( x264_nal_end( h ) )
            return -1;
        overhead += h->out.nal[h->out.i_nal-1].i_payload + NALU_OVERHEAD - (h->param.b_annexb && h->out.i_nal-1);
        if( h->fenc->extra_sei.sei_free && h->fenc->extra_sei.payloads[i].payload )
            h->fenc->extra_sei.sei_free( h->fenc->extra_sei.payloads[i].payload );
    }

    if( h->fenc->extra_sei.sei_free && h->fenc->extra_sei.payloads )
        h->fenc->extra_sei.sei_free( h->fenc->extra_sei.payloads );

    if( h->fenc->b_keyframe )
    {
        if( h->param.b_repeat_headers && h->fenc->i_frame == 0 && h->param.b_version_info )
        {
            /* identify ourself */
            x264_nal_start( h, NAL_SEI, NAL_PRIORITY_DISPOSABLE );
            if( x264_sei_version_write( h, &h->out.bs ) )
                return -1;
            if( x264_nal_end( h ) )
                return -1;
            overhead += h->out.nal[h->out.i_nal-1].i_payload + NALU_OVERHEAD - (h->param.b_annexb && h->out.i_nal-1);
        }

        if( h->fenc->i_type != X264_TYPE_IDR )
        {
            int time_to_recovery = h->param.i_open_gop ? 0 : X264_MIN( h->mb.i_mb_width - 1, h->param.i_keyint_max ) + h->param.i_bframe - 1;
            x264_nal_start( h, NAL_SEI, NAL_PRIORITY_DISPOSABLE );
            x264_sei_recovery_point_write( h, &h->out.bs, time_to_recovery );
            if( x264_nal_end( h ) )
                return -1;
            overhead += h->out.nal[h->out.i_nal-1].i_payload + NALU_OVERHEAD - (h->param.b_annexb && h->out.i_nal-1);
        }

        if ( h->param.i_frame_packing >= 0 )
        {
            x264_nal_start( h, NAL_SEI, NAL_PRIORITY_DISPOSABLE );
            x264_sei_frame_packing_write( h, &h->out.bs );
            if( x264_nal_end( h ) )
                return -1;
            overhead += h->out.nal[h->out.i_nal-1].i_payload + NALU_OVERHEAD - (h->param.b_annexb && h->out.i_nal-1);
        }
    }

    /* generate sei pic timing */
    if( h->sps->vui.b_pic_struct_present || h->sps->vui.b_nal_hrd_parameters_present )
    {
        x264_nal_start( h, NAL_SEI, NAL_PRIORITY_DISPOSABLE );
        x264_sei_pic_timing_write( h, &h->out.bs );
        if( x264_nal_end( h ) )
            return -1;
        overhead += h->out.nal[h->out.i_nal-1].i_payload + NALU_OVERHEAD - (h->param.b_annexb && h->out.i_nal-1);
    }

    if( h->fenc->b_keyframe && h->param.b_intra_refresh )
        h->i_cpb_delay_pir_offset = h->fenc->i_cpb_delay;

    /* Init the rate control */
    /* FIXME: Include slice header bit cost. */
    x264_ratecontrol_start( h, h->fenc->i_qpplus1, overhead*8 );
    i_global_qp = x264_ratecontrol_qp( h );

    pic_out->i_qpplus1 =
    h->fdec->i_qpplus1 = i_global_qp + 1;

    if( h->param.rc.b_stat_read && h->sh.i_type != SLICE_TYPE_I )
    {
        x264_reference_build_list_optimal( h );
        x264_reference_check_reorder( h );
    }

    if( h->i_ref0 )
        h->fdec->i_poc_l0ref0 = h->fref0[0]->i_poc;

    if( h->sh.i_type == SLICE_TYPE_B )
        x264_macroblock_bipred_init( h );

    /*------------------------- Weights -------------------------------------*/
    x264_weighted_pred_init( h );

    /* ------------------------ Create slice header  ----------------------- */
    x264_slice_init( h, i_nal_type, i_global_qp );

    if( i_nal_ref_idc != NAL_PRIORITY_DISPOSABLE )
        h->i_frame_num++;

#if mcdull_SWATAW
    swataw_set_type(McDull->swataw, h->sh.i_type);
    swataw_load(McDull->swataw, h->fenc->plane, h->fenc->i_stride, McDull->width, McDull->height);
    swataw_mode(McDull->swataw);
#endif

    /* Write frame */
    h->i_threadslice_start = 0;
    h->i_threadslice_end = h->mb.i_mb_height;
    if( (intptr_t)dull_slices_write( McDull ) )
        return -1;
//} _encoder_encode //} _encoder_encode //} _encoder_encode //} _encoder_encode

//{ _encoder_frame_end //{ _encoder_frame_end //{ _encoder_frame_end //{ _encoder_frame_end 

    if( !h->out.i_nal )
    {
        pic_out->i_type = X264_TYPE_AUTO;
        return 0;
    }

    /* generate sei buffering period and insert it into place */
    if( h->fenc->b_keyframe && h->sps->vui.b_nal_hrd_parameters_present )
    {
        int idx = 0;
        x264_nal_t nal_tmp;
        x264_hrd_fullness( h );
        x264_nal_start( h, NAL_SEI, NAL_PRIORITY_DISPOSABLE );
        x264_sei_buffering_period_write( h, &h->out.bs );
        if( x264_nal_end( h ) )
           return -1;
        /* buffering period sei must follow AUD, SPS and PPS and precede all other SEIs */
        while( h->out.nal[idx].i_type == NAL_AUD ||
               h->out.nal[idx].i_type == NAL_SPS ||
               h->out.nal[idx].i_type == NAL_PPS )
            idx++;
        nal_tmp = h->out.nal[h->out.i_nal-1];
        memmove( &h->out.nal[idx+1], &h->out.nal[idx], (h->out.i_nal-idx-1)*sizeof(x264_nal_t) );
        h->out.nal[idx] = nal_tmp;
    }

#if mcdull_SWATAW
    swataw_save(McDull->swataw);
#endif

    frame_size = x264_encoder_encapsulate_nals( h, 0 );

    /* Set output picture properties */
    pic_out->i_type = h->fenc->i_type;

    pic_out->b_keyframe = h->fenc->b_keyframe;

    pic_out->i_pts = h->fdec->i_pts;
    pic_out->i_dts = h->fdec->i_dts;

    pic_out->img.i_csp = X264_CSP_NV12;
    pic_out->img.i_plane = h->fdec->i_plane;
    for( i = 0; i < 2; i++ )
    {
        pic_out->img.i_stride[i] = h->fdec->i_stride[i] * sizeof(pixel);
        pic_out->img.plane[i] = (uint8_t*)h->fdec->plane[i];
    }

    x264_frame_push_unused( h, h->fenc );

    /* ---------------------- Update encoder state ------------------------- */

    /* update rc */
    if( x264_ratecontrol_end( h, frame_size * 8, &filler ) < 0 )
        return -1;

    pic_out->hrd_timing = h->fenc->hrd_timing;

    while( filler > 0 )
    {
        int f, total_size;
        overhead = (FILLER_OVERHEAD - h->param.b_annexb);
        if( h->param.i_slice_max_size && filler > h->param.i_slice_max_size )
        {
            int next_size = filler - h->param.i_slice_max_size;
            int overflow = X264_MAX( overhead - next_size, 0 );
            f = h->param.i_slice_max_size - overhead - overflow;
        }
        else
            f = X264_MAX( 0, filler - overhead );

        x264_nal_start( h, NAL_FILLER, NAL_PRIORITY_DISPOSABLE );
        x264_filler_write( h, &h->out.bs, f );
        if( x264_nal_end( h ) )
            return -1;
        total_size = x264_encoder_encapsulate_nals( h, h->out.i_nal-1 );
        frame_size += total_size;
        filler -= total_size;
    }

    /* End bitstream, set output  */
    *pi_nal = h->out.i_nal;
    *pp_nal = h->out.nal;

    h->out.i_nal = 0;

    x264_noise_reduction_update( h );

    /* Slice stat */
    h->stat.i_frame_count[h->sh.i_type]++;
    h->stat.i_frame_size[h->sh.i_type] += frame_size;
    h->stat.f_frame_qp[h->sh.i_type] += h->fdec->f_qp_avg_aq;

    for( i = 0; i < X264_MBTYPE_MAX; i++ )
        h->stat.i_mb_count[h->sh.i_type][i] += h->stat.frame.i_mb_count[i];
    for( i = 0; i < X264_PARTTYPE_MAX; i++ )
        h->stat.i_mb_partition[h->sh.i_type][i] += h->stat.frame.i_mb_partition[i];
    for( i = 0; i < 2; i++ )
        h->stat.i_mb_count_8x8dct[i] += h->stat.frame.i_mb_count_8x8dct[i];
    for( i = 0; i < 6; i++ )
        h->stat.i_mb_cbp[i] += h->stat.frame.i_mb_cbp[i];
    for( i = 0; i < 4; i++ )
        for( j = 0; j < 13; j++ )
            h->stat.i_mb_pred_mode[i][j] += h->stat.frame.i_mb_pred_mode[i][j];
    if( h->sh.i_type != SLICE_TYPE_I )
        for( i_list = 0; i_list < 2; i_list++ )
            for( i = 0; i < X264_REF_MAX*2; i++ )
                h->stat.i_mb_count_ref[h->sh.i_type][i_list][i] += h->stat.frame.i_mb_count_ref[i_list][i];
    if( h->sh.i_type == SLICE_TYPE_P )
    {
        h->stat.i_consecutive_bframes[h->fdec->i_frame - h->fref0[0]->i_frame - 1]++;
        if( h->param.analyse.i_weighted_pred >= X264_WEIGHTP_SIMPLE )
        {
            h->stat.i_wpred[0] += !!h->sh.weight[0][0].weightfn;
            h->stat.i_wpred[1] += !!h->sh.weight[0][1].weightfn || !!h->sh.weight[0][2].weightfn;
        }
    }
    if( h->sh.i_type == SLICE_TYPE_B )
    {
        h->stat.i_direct_frames[ h->sh.b_direct_spatial_mv_pred ] ++;
        if( h->mb.b_direct_auto_write )
        {
            //FIXME somewhat arbitrary time constants
            if( h->stat.i_direct_score[0] + h->stat.i_direct_score[1] > h->mb.i_mb_count )
                for( i = 0; i < 2; i++ )
                    h->stat.i_direct_score[i] = h->stat.i_direct_score[i] * 9/10;
            for( i = 0; i < 2; i++ )
                h->stat.i_direct_score[i] += h->stat.frame.i_direct_score[i];
        }
    }

    psz_message[0] = '\0';
    dur = h->fenc->f_duration;
    h->stat.f_frame_duration[h->sh.i_type] += dur;
    if( h->param.analyse.b_psnr )
    {
        int64_t ssd[3] =
        {
            h->stat.frame.i_ssd[0],
            h->stat.frame.i_ssd[1],
            h->stat.frame.i_ssd[2],
        };

        h->stat.f_ssd_global[h->sh.i_type]   += dur * (ssd[0] + ssd[1] + ssd[2]);
        h->stat.f_psnr_average[h->sh.i_type] += dur * x264_psnr( ssd[0] + ssd[1] + ssd[2], 3 * h->param.i_width * h->param.i_height / 2 );
        h->stat.f_psnr_mean_y[h->sh.i_type]  += dur * x264_psnr( ssd[0], h->param.i_width * h->param.i_height );
        h->stat.f_psnr_mean_u[h->sh.i_type]  += dur * x264_psnr( ssd[1], h->param.i_width * h->param.i_height / 4 );
        h->stat.f_psnr_mean_v[h->sh.i_type]  += dur * x264_psnr( ssd[2], h->param.i_width * h->param.i_height / 4 );

        snprintf( psz_message, 80, " PSNR Y:%5.2f U:%5.2f V:%5.2f",
                  x264_psnr( ssd[0], h->param.i_width * h->param.i_height ),
                  x264_psnr( ssd[1], h->param.i_width * h->param.i_height / 4),
                  x264_psnr( ssd[2], h->param.i_width * h->param.i_height / 4) );
    }

    if( h->param.analyse.b_ssim )
    {
        double ssim_y = h->stat.frame.f_ssim
                      / (((h->param.i_width-6)>>2) * ((h->param.i_height-6)>>2));
        h->stat.f_ssim_mean_y[h->sh.i_type] += ssim_y * dur;
        snprintf( psz_message + strlen(psz_message), 80 - strlen(psz_message),
                  " SSIM Y:%.5f", ssim_y );
    }
    psz_message[79] = '\0';

    x264_log( h, X264_LOG_DEBUG,
                  "frame=%4d QP=%.2f NAL=%d Slice:%c Poc:%-3d I:%-4d P:%-4d SKIP:%-4d size=%d bytes%s\n",
              h->i_frame,
              h->fdec->f_qp_avg_aq,
              h->i_nal_ref_idc,
              h->sh.i_type == SLICE_TYPE_I ? 'I' : (h->sh.i_type == SLICE_TYPE_P ? 'P' : 'B' ),
              h->fdec->i_poc,
              h->stat.frame.i_mb_count_i,
              h->stat.frame.i_mb_count_p,
              h->stat.frame.i_mb_count_skip,
              frame_size,
              psz_message );

    /* Remove duplicates, must be done near the end as breaks h->fref0 array
     * by freeing some of its pointers. */
    for( i = 0; i < h->i_ref0; i++ )
        if( h->fref0[i] && h->fref0[i]->b_duplicate )
        {
            x264_frame_push_blank_unused( h, h->fref0[i] );
            h->fref0[i] = 0;
        }

    if( h->param.psz_dump_yuv )
        x264_frame_dump( h );

    return frame_size;
}
//} _encoder_frame_end //} _encoder_frame_end //} _encoder_frame_end //} _encoder_frame_end 


/****************************************************************************
 * x264_encoder_close:
 ****************************************************************************/
/****************************************************************************
 ****************************************************************************/

#endif // _MCDULL_ENCODER_
