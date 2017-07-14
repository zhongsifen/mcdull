// swataw_core.h

#ifndef _swataw_core_
#define _swataw_core_ 1

#include "swataw.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
	int32_t bp_val[4*4];
	int32_t bp_dif[4*4];

	int32_t mb_a1;
	int32_t mb_aa;

    int32_t mb_var;
	int32_t mb_dev;

	int32_t mb_sad;
	int32_t mb_mad;
}
swataw_data_t;

typedef struct
{
    swataw_data_t* data;

	swataw_mode_t mode;
}
swataw_mb_t;

typedef struct
{
	int n1, n2, n;
	int32_t nump;
	swataw_type_t type;
	swataw_mb_t* cur;
	swataw_mb_t* ref;

	//{
	swataw_data_t* _data[2];
	swataw_mb_t* _mb[2];
	//}
}
swataw_t;

void
swataw_mb_load(swataw_t* swataw, int k1, int k2,
		uint8_t* plane[3], int stride[3])
;
void 
swataw_mb_save(swataw_t* swataw, int k1, int k2)
;
void 
swataw_mb_stat(swataw_t* swataw, int k1, int k2)
;
void 
swataw_mb_mode(swataw_t* swataw, int k1, int k2)
;
void 
swataw_I_mb_mode(swataw_t* swataw, int k1, int k2)
;
void 
swataw_P_mb_mode(swataw_t* swataw, int k1, int k2)
;
void 
swataw_B_mb_mode(swataw_t* swataw, int k1, int k2)
;

#ifdef __cplusplus
}
#endif

#endif // _swataw_core_
