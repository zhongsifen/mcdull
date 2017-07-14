// swataw_core.c

#include "swataw_core.h"

#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include <math.h>

void _swap(void** p1, void** p2)
{
    void* q = *p1; *p1 = *p2; *p2 = q;
}

int32_t _max(int32_t a1, int32_t a2)
{
    return a1 > a2 ? a1 : a2;
}

void
swataw_mb_load(swataw_t* swataw, int k1, int k2,
		uint8_t* plane[3], int stride[3])
{
    int l1 = stride[0];
	int n1 = swataw->n1;
	int k = k1 + k2*n1;
	uint8_t* y_k = plane[0] + (k1<<4) + (k2<<4)*l1;
	int32_t* bp_k = swataw->cur[k].data->bp_val;
	int i1, i2;
	int j1, j2, j;
	uint8_t* f;
	int16_t a;

	for (j=0,j2=0; j2<4; j2++) {
		for (j1=0; j1<4; j1++,j++) {
			f = y_k + (j1<<2) + (j2<<2)*l1;
			a = 0;
			for (    i2=0; i2<4; i2++) {
				for (i1=0; i1<4; i1++) {
					a += f[i1];
				}
				f += l1;
			}
			bp_k[j] = a;
		}
	}

	swataw->cur[k].mode = swataw_MODE_X_BEST;
}

void 
swataw_mb_save(swataw_t* swataw, int k1, int k2)
{
	int n1 = swataw->n1;
	int k = k1 + k2*n1;

    if (swataw->cur[k].mode == swataw_MODE_P_SKIP)
    {
        _swap((void**)&swataw->cur[k].data, (void**)&swataw->ref[k].data);
    }
}

void
swataw_mb_stat(swataw_t* swataw, int k1, int k2) 
{
	int k = k1 + k2*swataw->n1;
	swataw_data_t* data_cur = swataw->cur[k].data;
    swataw_data_t* data_ref = swataw->ref[k].data;
	int32_t* val_cur = data_cur->bp_val;
	int32_t* val_ref = data_ref->bp_val;

	int32_t d0 = 0;
	int32_t sd0 = 0;
	int32_t md0 = 0;

	int i;

	for (i=0; i<16; i++) {
		d0 = abs(val_cur[i] - val_ref[i]);
		sd0 += d0;
		md0 = _max(md0, d0);
	}
	data_cur->mb_mad = md0;
	data_cur->mb_sad = sd0;
}

void
swataw_I_mb_mode(swataw_t* swataw, int k1, int k2) 
{
	int n1 = swataw->n1;
	int k = k1 + k2*n1;
	swataw_mb_t* mb_cur = &swataw->cur[k];
	swataw_mb_t* mb_ref = &swataw->ref[k];
    swataw_data_t* data_cur = mb_cur->data;
    swataw_data_t* data_ref = mb_ref->data;

    if (data_cur->mb_var < 64*64) 
    {
		mb_cur->mode = swataw_MODE_I_FAST;
	}
	else 
    {
		mb_cur->mode = swataw_MODE_I_BEST;
	}
}

void
swataw_P_mb_mode(swataw_t* swataw, int k1, int k2) 
{
	int n1 = swataw->n1;
	int k = k1 + k2*n1;
	swataw_mb_t* mb_cur = &swataw->cur[k];
	swataw_mb_t* mb_ref = &swataw->ref[k];
    swataw_data_t* data_cur = mb_cur->data;
    swataw_data_t* data_ref = mb_ref->data;

	if (data_cur->mb_sad < 1024) 
    {
		mb_cur->mode = swataw_MODE_P_SKIP;
	}
	//else 
	//if (data_cur->mb_sad < 640) 
	//{
	//	mb_cur->mode = swataw_MODE_P_FAST;
	//}
	//else 
	//if (data_cur->mb_sad < 640) 
	//{
	//	mb_cur->mode = swataw_MODE_P_GOOD;
	//}
	else 
	{
		mb_cur->mode = swataw_MODE_P_BEST;
	}
}

void
swataw_B_mb_mode(swataw_t* swataw, int k1, int k2) 
{
	int n1 = swataw->n1;
	int k = k1 + k2*n1;
	swataw_mb_t* mb_cur = &swataw->cur[k];
	//swataw_mb_t* mb_ref = &swataw->ref[k];
	//swataw_data_t* data_cur = mb_cur->data;
	//swataw_data_t* data_ref = mb_ref->data;

    if (mb_cur->mode == swataw_MODE_P_SKIP)
    {
		mb_cur->mode =  swataw_MODE_B_SKIP;
	}
	else 
    if (mb_cur->mode == swataw_MODE_P_FAST)
    {
		mb_cur->mode =  swataw_MODE_B_FAST;
	}
	else 
    if (mb_cur->mode == swataw_MODE_P_GOOD)
    {
		mb_cur->mode =  swataw_MODE_B_GOOD;
	}
	else 
	{
		mb_cur->mode =  swataw_MODE_B_BEST;
	}
}

void
swataw_mb_mode(swataw_t* swataw, int k1, int k2) 
{
    switch (swataw->type)
    {
    case swataw_TYPE_I:
        swataw_I_mb_mode(swataw, k1, k2);
        break;
    case swataw_TYPE_P:
        swataw_P_mb_mode(swataw, k1, k2);
        break;
    case swataw_TYPE_B:
        swataw_B_mb_mode(swataw, k1, k2);
        break;
    default:
        break;
    }
}
