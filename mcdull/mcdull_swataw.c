#include "mcdull_core.h"
#if mcdull_SWATAW

#include "encoder/analyse.h"
#include "encoder/macroblock.h"
#include "encoder/ratecontrol.h"

void McDull_swataw_X_BEST( McDull_t* McDull )
{
    x264_t* h = McDull->x264;

    x264_macroblock_analyse( h );
    x264_macroblock_encode( h );
}

void McDull_swataw( McDull_t* McDull )
{
    x264_t* h = McDull->x264;

    int i_mb_x = h->mb.i_mb_x;
    int i_mb_y = h->mb.i_mb_y;

    swataw_mode_t mode = swataw_get_mode(McDull->swataw, i_mb_x, i_mb_y);

    h->mb.i_qp = x264_ratecontrol_mb_qp( h );

    if (mode == swataw_MODE_P_SKIP)
        if ( M32(h->mb.cache.pskip_mv) )
        {
            mode = swataw_MODE_P_FAST;
            swataw_set_mode(McDull->swataw, mode, i_mb_x, i_mb_y);
        }
    if (mode == swataw_MODE_B_SKIP)
        if ( M32(h->mb.cache.pskip_mv) )
        {
            mode = swataw_MODE_B_FAST;
            swataw_set_mode(McDull->swataw, mode, i_mb_x, i_mb_y);
        }

    switch(mode)
    {
    case swataw_MODE_P_SKIP:
        McDull_swataw_P_SKIP( McDull );
        break;
    case swataw_MODE_P_FAST:
        McDull_swataw_P_FAST( McDull );
        break;
    case swataw_MODE_P_GOOD:
        //McDull_swataw_P_GOOD( McDull );
        //break;
    case swataw_MODE_P_BEST:
        McDull_swataw_P_BEST( McDull );
        break;

        //McDull_swataw_X_BEST( McDull );
        //break;

    case swataw_MODE_B_SKIP:
        //McDull_swataw_B_SKIP( McDull );
        //break;
    case swataw_MODE_B_FAST:
        //McDull_swataw_B_FAST( McDull );
        //break;
    case swataw_MODE_B_GOOD:
        //McDull_swataw_B_GOOD( McDull );
        //break;
    case swataw_MODE_B_BEST:
        //McDull_swataw_B_GOOD( McDull );
        //break;

        McDull_swataw_X_BEST( McDull );
        break;

    case swataw_MODE_I_FAST:
        //McDull_swataw_I_FAST( McDull );
        //break;
    case swataw_MODE_I_BEST:
        //McDull_swataw_I_BEST( McDull );
        //break;

        McDull_swataw_X_BEST( McDull );
        break;

	default:
        assert( 0 );
        break;
    }
}

#endif // mcdull_SWATAW
