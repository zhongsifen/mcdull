// mcdull.c

#include "mcdull.h"
#include "mcdull_core.h"

void* mcdull_new()
{
	void*p = malloc(sizeof(McDull_t));
	memset(p, 0, sizeof(McDull_t));

	return p;
}

void mcdull_delete(void* p)
{
	free(p);
}

int mcdull_open(void* p,
		int width, int height)
{
    McDull_t* McDull = (McDull_t*)p;

    McDull_open(McDull, width, height);

    return 0;
}

int mcdull_close(void* p)
{
    McDull_t* McDull = (McDull_t*)p;

    McDull_close(McDull);

    return 0;
}

int mcdull_set_qp(void* p,
		int qp)
{
    McDull_t* McDull = (McDull_t*)p;

    McDull_set_qp(McDull, qp);

    return 0;
}

int mcdull_set_keyint( void* p,
		int keyint)
{
    McDull_t* McDull = (McDull_t*)p;

    McDull_set_keyint(McDull, keyint);

    return 0;
}

int mcdull_set_bframe( void* p,
		int bframe)
{
    McDull_t* McDull = (McDull_t*)p;

    McDull_set_bframe(McDull, bframe);

    return 0;
}

int mcdull_encode_header(void* p,
		int32_t g_264[], int32_t g_264_param[])
{
    McDull_t* McDull = (McDull_t*)p;
    
    int32_t g_264_size[1];

    McDull_encode_header(McDull, (uint8_t*)g_264, g_264_size);
    g_264_param[0] = g_264_size[0];

    return g_264_size[0];
}

int mcdull_encode(void* p,
		int32_t f_yuv[],
		int i_pts,
		int32_t g_264[], int32_t g_264_param[],
		int32_t g_yuv[])
{
    McDull_t* McDull = (McDull_t*)p;

    int32_t g_264_size[1];

    McDull_encode(McDull, (uint8_t*)f_yuv, i_pts, (uint8_t*)g_264, g_264_size, (uint8_t*)g_yuv);
    g_264_param[0] = g_264_size[0];

    return g_264_size[0];
}

int mcdull_encode_flush(void* p,
		int32_t g_264[], int32_t g_264_param[],
		int32_t g_yuv[])
{
    McDull_t* McDull = (McDull_t*)p;

    int32_t g_264_size[1];

    McDull_encode_flush(McDull, (uint8_t*)g_264, g_264_size, (uint8_t*)g_yuv);
    g_264_param[0] = g_264_size[0];

    return g_264_size[0];
}
