/*
 * This file contains implementations of the divisions
 * DivU32U16()
 * DivW32W16()
 *
 *
 * The description header can be found in signal_processing_library.h
 *
 */

#include "signal_processing_library.h"

uint32_t DivU32U16(uint32_t num, uint16_t den)
{
    // Guard against division with 0
    if (den != 0)
    {
        return (uint32_t)(num / den);
    } else
    {
        return (uint32_t)0xFFFFFFFF;
    }
}

int32_t DivW32W16(int32_t num, int16_t den)
{
    // Guard against division with 0
    if (den != 0)
    {
        return (int32_t)(num / den);
    } else
    {
        return (int32_t)0x7FFFFFFF;
    }
}
