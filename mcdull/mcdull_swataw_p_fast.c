#include "mcdull_core.h"

#if mcdull_SWATAW

#include "encoder/macroblock.h"

void McDull_swataw_P_FAST( McDull_t* McDull )
{
    x264_t* h = McDull->x264;

    dull_macroblock_analyse_P_FAST( h );
    dull_macroblock_encode_P( h );
}

#endif
