// swataw.c

#include "swataw_core.h"

#include <stdlib.h>
#include <memory.h>
#include <assert.h>

void _swap(void** p1, void** p2)
;

void* swataw_new()
{
	void* st = malloc(sizeof(swataw_t));
	memset(st, 0, sizeof(swataw_t));
	return st;
}

void swataw_delete(void* st)
{
	free(st);
}

swataw_type_t
swataw_get_type(void* st)
{
	swataw_t* swataw = (swataw_t*)st;
	return swataw->type;
}

void
swataw_set_type(void* st,
		swataw_type_t type)
{
	swataw_t* swataw = (swataw_t*)st;
	swataw->type = type;
}

swataw_mb_t*
swataw_get_mb(void* st,
		int k1, int k2)
{
	swataw_t* swataw = (swataw_t*)st;
	return &swataw->cur[k1 + k2*swataw->n1];
}

swataw_mode_t
swataw_get_mode(void* st,
		int k1, int k2)
{
	swataw_t* swataw = (swataw_t*)st;
	swataw_mode_t mode = swataw->cur[k1 + k2*swataw->n1].mode;

	return mode;
}

void
swataw_set_mode(void* st,
        swataw_mode_t mode,
		int k1, int k2)
{
	swataw_t* swataw = (swataw_t*)st;
	swataw->cur[k1 + k2*swataw->n1].mode = mode;
}

void
swataw_open(void* st, 
		int l1, int l2)
{
	swataw_t* swataw = (swataw_t*)st;
	int n1 = l1>>4;
	int n2 = l2>>4;
	int n = n1*n2;
	int k;

	swataw->n1 = n1;
	swataw->n2 = n2;
	swataw->n  = n;

	swataw->nump = -1;

	swataw->_mb[0] = (swataw_mb_t*)malloc(sizeof(swataw_mb_t)*n);
	swataw->_mb[1] = (swataw_mb_t*)malloc(sizeof(swataw_mb_t)*n);
	memset(swataw->_mb[0], 0, sizeof(swataw_mb_t)*n);
	memset(swataw->_mb[1], 0, sizeof(swataw_mb_t)*n);
	swataw->_data[0] = (swataw_data_t*)malloc(sizeof(swataw_data_t)*n);
	swataw->_data[1] = (swataw_data_t*)malloc(sizeof(swataw_data_t)*n);
	memset(swataw->_data[0], 0, sizeof(swataw_data_t)*n);
	memset(swataw->_data[1], 0, sizeof(swataw_data_t)*n);
	for (k=0; k<n; k++) {
		swataw->_mb[0][k].data = &swataw->_data[0][k];
		swataw->_mb[1][k].data = &swataw->_data[1][k];
	}
	swataw->cur = swataw->_mb[0];
	swataw->ref = swataw->_mb[1];

    swataw->type = swataw_TYPE_I;
}

void
swataw_close(void* st)
{
	swataw_t* swataw = (swataw_t*)st;

	free(swataw->_mb[0]);
	free(swataw->_mb[1]);
	free(swataw->_data[0]);
	free(swataw->_data[1]);
}

void
swataw_load(void* st,
        uint8_t* plane[3], int stride[3],
		int l1, int l2)
{
	swataw_t* swataw = (swataw_t*)st;
	int m1 = l1>>1;
	int m2 = l2>>1;
	int n1 = swataw->n1;
	int n2 = swataw->n2;
	int k1, k2;

    if (swataw->type == swataw_TYPE_B) return;

    _swap((void**)&swataw->cur, (void**)&swataw->ref);

	switch (swataw->type)
	{
	case swataw_TYPE_I:
		swataw->nump = 0;
		break;
	case swataw_TYPE_P:
		swataw->nump++;
		break;
	case swataw_TYPE_B:
		return;
	default:
		assert(0);
	}

	for(    k2=0; k2 < n2; k2++) {
		for(k1=0; k1 < n1; k1++) {
			swataw_mb_load(swataw, k1, k2, plane, stride);
		}
	}
}

void
swataw_save(void* st)
{
	swataw_t* swataw = (swataw_t*)st;
	int n1 = swataw->n1;
	int n2 = swataw->n2;
	int k1, k2;

    if (swataw->type == swataw_TYPE_B) return;

	for(    k2=0; k2 < n2; k2++) {
		for(k1=0; k1 < n1; k1++) {
			swataw_mb_save(swataw, k1, k2);
		}
	}
}

void
swataw_type(void* st)
{
}

void
swataw_mode(void* st)
{
	swataw_t* swataw = (swataw_t*)st;
	int n1 = swataw->n1;
	int n2 = swataw->n2;
	int k1, k2;

	for(    k2=0; k2 < n2; k2++) {
		for(k1=0; k1 < n1; k1++) {
			swataw_mb_stat(swataw, k1, k2);
			swataw_mb_mode(swataw, k1, k2);
		}
	}
}
