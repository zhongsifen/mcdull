// app.cpp

#include "mcdull/mcdull.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FRAME_TOTAL 200

int fopen_safe(FILE** _FILE, const char* _Filename, const char* _Mode)
{
#ifdef _MSC_VER
    return fopen_s(_FILE, _Filename, _Mode);
#else
    *_FILE = fopen(_Filename, _Mode);
    if (*_FILE == NULL) return -1;

    return 0;
#endif
}

static int32_t f_yuv[0x1000000];
static int32_t g_264[0x10000];
static int32_t g_yuv[0x1000000];

int main_app(int argc, char* argv[])
{
	char name[80] = {"../../work/data/paris"};
	int l1 = 352;
	int l2 = 288;

	int k=1;
	if (k < argc) sprintf(name, "%s", argv[k++]);
	if (k < argc) l1 = atoi(argv[k++]);
	if (k < argc) l2 = atoi(argv[k++]);

	int32_t qp = 16 + 6*2;
	int32_t keyint = 50;
	int32_t bframe = 0;

	int l = l1*l2;
	int m = l>>2;
	int lmm = l + m + m;

	char filename[80];
	char filename_f_yuv[80];
	//char filename_h_yuv[80];
	char filename_g_264[80];
	char filename_g_yuv[80];

    FILE* file_f_yuv;
    //FILE* file_h_yuv;
    FILE* file_g_264;
    //FILE* file_g_yuv;

    int g_264_size = 0;

	void* mcdull = mcdull_new();

    int ret = 0;

	sprintf(filename, "%s_%dx%d", name, l1, l2);

	sprintf(filename_f_yuv, "%s.yuv",        filename);
	//sprintf(filename_h_yuv, "%s_h.yuv",      filename);
#if mcdull_SWATAW
	sprintf(filename_g_264, "%s_mcdull.264", filename);
	sprintf(filename_g_yuv, "%s_mcdull.yuv", filename);
	//sprintf(filename_g_mp4, "%s_mcdull.mp4", filename);
#else
	sprintf(filename_g_264, "%s_x264.264", filename);
	sprintf(filename_g_yuv, "%s_x264.yuv", filename);
	//sprintf(filename_g_mp4, "%s_x264.mp4", filename);
#endif
	mcdull_set_qp(mcdull, qp);
	mcdull_set_keyint(mcdull, keyint);
	mcdull_set_bframe(mcdull, bframe);

	ret = mcdull_open(mcdull, l1, l2);						if (ret < 0) return -1;

    ret = fopen_safe(&file_f_yuv, filename_f_yuv, "rb");   if (ret != 0) return -1;
    //ret = fopen_safe(&file_h_yuv, filename_h_yuv, "wb");   if (ret != 0) return -1;
    ret = fopen_safe(&file_g_264, filename_g_264, "wb");   if (ret != 0) return -1;
    //ret = fopen_safe(&file_g_yuv, filename_g_yuv, "wb");   if (ret != 0) return -1;

    int i_frame_total = FRAME_TOTAL;
    int i_frame, i_frame_output;

    for( i_frame = 0, i_frame_output = 0; i_frame < i_frame_total; i_frame++ )
    {
		ret = fread (f_yuv, 1, lmm, file_f_yuv);	if (ret < 0) goto done;
		//ret = fwrite(f_yuv, 1, lmm, file_h_yuv);	if (ret < 0) goto done;

		g_264_size = mcdull_encode(mcdull, f_yuv, i_frame, &g_264[1], &g_264[0], g_yuv);
        { int i; for (i=0; i<80; i++) printf("\b"); }
		printf("%4d: size = %6d", i_frame, g_264_size);

        if (g_264_size < 0)
            return -1;
        if (g_264_size > 0)
        {
			ret = fwrite(&g_264[1], 1, g_264_size, file_g_264);   if (ret < 0) goto done;
			//ret = fwrite(&g_yuv[0], 1, lmm,        file_g_yuv);   if (ret < 0) goto done;
            i_frame_output++;
        }
    }

	do
	{
		g_264_size = mcdull_encode_flush(mcdull, &g_264[1], &g_264[0], g_yuv);
        { int i; for (i=0; i<80; i++) printf("\b"); }
		printf("%4d: size = %6d", i_frame, g_264_size);

		if (g_264_size > 0)
		{
			ret = fwrite(&g_264[1], 1, g_264_size, file_g_264);   if (ret < 0) goto done;
			//ret = fwrite(&g_yuv[0], 1, lmm,        file_g_yuv);   if (ret < 0) goto done;
            i_frame_output++;
		}
		i_frame++;
	} while( g_264_size > 0 );

done:
	printf("\n");

	mcdull_close(mcdull);
	mcdull_delete(mcdull);

	fclose(file_f_yuv);
	//fclose(file_h_yuv);
	fclose(file_g_264);
	//fclose(file_g_yuv);

	return 0;
}
