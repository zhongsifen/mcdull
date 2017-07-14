// swataw.h

#ifndef _swataw_
#define _swataw_ 1

#include "common.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void* swataw_new()
;

void swataw_delete(void* st)
;

// mcdull/common/common.h
//
//enum slice_type_e
//{
//    SLICE_TYPE_P  = 0,
//    SLICE_TYPE_B  = 1,
//    SLICE_TYPE_I  = 2,
//    SLICE_TYPE_SP = 3,
//    SLICE_TYPE_SI = 4
//};
typedef enum
{
    swataw_TYPE_P  = SLICE_TYPE_P,		// = 0,
    swataw_TYPE_B  = SLICE_TYPE_B,		// = 1,
    swataw_TYPE_I  = SLICE_TYPE_I,		// = 2,
}
swataw_type_t;

typedef enum 
{
	swataw_MODE_0_NONE = 0x00,

	swataw_MODE_P_SKIP = 0x01,
	swataw_MODE_P_FAST = 0x02,
	swataw_MODE_P_GOOD = 0x03,
	swataw_MODE_P_BEST = 0x0F,

	swataw_MODE_B_SKIP = 0x11,
	swataw_MODE_B_FAST = 0x12,
	swataw_MODE_B_GOOD = 0x13,
	swataw_MODE_B_BEST = 0x1F,

    swataw_MODE_I_SKIP = 0x21,
	swataw_MODE_I_FAST = 0x22,
	swataw_MODE_I_GOOD = 0x23,
	swataw_MODE_I_BEST = 0x2F,

	swataw_MODE_X_BEST = 0xFF,
}
swataw_mode_t;

swataw_type_t
swataw_get_type(void* st)
;
void
swataw_set_type(void* st,
		swataw_type_t type)
;
swataw_mode_t
swataw_get_mode(void* st,
		int k1, int k2)
;
void
swataw_set_mode(void* st,
        swataw_mode_t mode,
		int k1, int k2)
;
void
swataw_open(void* st, 
		int width, int height)
;
void
swataw_close(void* st)
;
void
swataw_load(void* st,
        uint8_t* plane[3], int stride[3],
		int l1, int l2)
;
void
swataw_save(void* st)
;
void
swataw_type(void* st)
;
void
swataw_mode(void* st)
;

#ifdef __cplusplus
}
#endif

#endif //_swataw_
