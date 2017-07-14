// mcdull_core.c

#include "mcdull_core.h"

int McDull_open(McDull_t* McDull,
        int width, int height)
{
    x264_t* x264 = McDull->x264;
    x264_param_t* param = McDull->param;
    x264_param_t* p = param;

    char* preset  = "medium";	// default: "medium";
    char* tune    = "psnr";		//
    char* profile = "main";		// default: "high"

    McDull->width  = width;
    McDull->height = height;

    x264_picture_init (McDull->pic);
    x264_picture_alloc(McDull->pic, X264_CSP_I420, width, height);
    x264_picture_init (McDull->pic_out);
    x264_picture_alloc(McDull->pic_out, X264_CSP_I420, width, height);

    x264_param_default_preset( param, preset, tune );
    x264_param_apply_fastfirstpass( param );
    x264_param_apply_profile( param, profile );

    param->i_width  = width;
    param->i_height = height;

    //OPT("keyint")
    {
        p->i_keyint_max = McDull->keyint;
        p->i_keyint_min = p->i_keyint_max;
    }
    //OPT2("qp", "qp_constant")
    {
        p->rc.i_qp_constant = McDull->qp;
        p->rc.i_rc_method = X264_RC_CQP;
    }
    //OPT("bframes")
    {
        p->i_bframe = McDull->bframe;
    }

    McDull->x264 = x264_encoder_open(param);		if( McDull->x264 == NULL ) return -1;
    x264_encoder_parameters(McDull->x264, param);

#if mcdull_SWATAW
	McDull->swataw = swataw_new();
    swataw_open(McDull->swataw, width, height);
#endif

    return 0;
}

int McDull_close(McDull_t* McDull)
{
    x264_t *h = McDull->x264;

#if mcdull_SWATAW
    swataw_close(McDull->swataw);
	swataw_delete(McDull->swataw);
#endif

    x264_encoder_close(h);

    return 0;
}

int McDull_get_qp(McDull_t* McDull)
{
	int qp = McDull->qp;

	return qp;
}

int McDull_set_qp(McDull_t* McDull,
		int qp)
{
	McDull->qp = qp;

	return qp;
}

int McDull_set_keyint(McDull_t* McDull,
		int keyint)
{
	McDull->keyint = keyint;

	return keyint;
}

int McDull_set_bframe(McDull_t* McDull,
		int bframe)
{
	McDull->bframe = bframe;

    return bframe;
}

void yuvI420_to_picI420(uint8_t* yuv, int l1, int l2, x264_picture_t* pic)
{
    int m1 = l1>>1;
    int m2 = l2>>1;
    int l = l1*l2;
    int m = l>>2;

    uint8_t* y = yuv;
    uint8_t* u = y + l;
    uint8_t* v = u + m;
    x264_image_t* image = &pic->img;

    x264_plane_copy_c(image->plane[0], image->i_stride[0], y, l1, l1, l2);
    x264_plane_copy_c(image->plane[1], image->i_stride[1], u, m1, m1, m2);
    x264_plane_copy_c(image->plane[2], image->i_stride[2], v, m1, m1, m2);
}

void picNV12_to_yuvI420(x264_picture_t* pic, int l1, int l2, uint8_t* yuv)
{
    int m1 = l1>>1;
    int m2 = l2>>1;
    int l = l1*l2;
    int m = l>>2;

    uint8_t* y = yuv;
    uint8_t* u = y + l;
    uint8_t* v = u + m;
    x264_image_t* image = &pic->img;

    x264_plane_copy_c(y, l1, image->plane[0], image->i_stride[0], l1, l2);
    x264_plane_copy_deinterleave_c(u, m1, v, m1, image->plane[1], image->i_stride[1], m1, m2);
}

int McDull_encode_header(McDull_t* McDull,
        uint8_t g_264[], int32_t g_264_size[1])
{
    int i_header_size = 0;

#if mcdull_SWATAW
	i_header_size = dull_encoder_headers(McDull->x264, &McDull->p_nal, &McDull->i_nal);
#else
	i_header_size = x264_encoder_headers(McDull->x264, &McDull->p_nal, &McDull->i_nal);
#endif
    g_264_size[0] = i_header_size;
    if( i_header_size > 0 )
    {
        memcpy(g_264, McDull->p_nal[0].p_payload, i_header_size);
    }

	return i_header_size;
}

int McDull_encode(McDull_t* McDull,
        uint8_t f_yuv[], 
        int pts,
        uint8_t g_264[], int32_t g_264_size[1], 
        uint8_t g_yuv[])
{
    x264_t *h = McDull->x264;
    int width  = McDull->width;
    int height = McDull->height;

    int i_frame_size = 0;

    x264_picture_t* f_pic = McDull->pic;
    x264_picture_t* g_pic = McDull->pic_out;

    yuvI420_to_picI420(f_yuv, width, height, f_pic);
    f_pic->i_pts = pts;
#if mcdull_SWATAW
    i_frame_size = dull_encoder_encode( McDull, &McDull->p_nal, &McDull->i_nal, f_pic, g_pic );
#else
	i_frame_size = x264_encoder_encode( McDull->x264, &McDull->p_nal, &McDull->i_nal, f_pic, g_pic );
#endif
    g_264_size[0] = i_frame_size;
    if (i_frame_size < 0) return -1;
    if( i_frame_size > 0 )
    {
        memcpy(g_264, McDull->p_nal[0].p_payload, i_frame_size);
        picNV12_to_yuvI420(g_pic, width, height, g_yuv);
    }

    return i_frame_size;
}

int McDull_encode_flush(McDull_t* McDull,
		uint8_t g_264[], int32_t g_264_size[1], uint8_t g_yuv[])
{
    int width  = McDull->width;
    int height = McDull->height;

    int i_frame_size = 0;

    x264_picture_t* g_pic = McDull->pic_out;
    x264_picture_t* f_pic = NULL;

#if mcdull_SWATAW
    i_frame_size = dull_encoder_encode( McDull, &McDull->p_nal, &McDull->i_nal, f_pic, g_pic );
#else
	i_frame_size = x264_encoder_encode( McDull->x264, &McDull->p_nal, &McDull->i_nal, f_pic, g_pic );
#endif
    g_264_size[0] = i_frame_size;
    if( i_frame_size > 0 )
    {
        memcpy(g_264, McDull->p_nal[0].p_payload, i_frame_size);
        picNV12_to_yuvI420(g_pic, width, height, g_yuv);
        McDull->frame_out++;
    }

    if (i_frame_size < 0) return -1;

    return i_frame_size;
}
