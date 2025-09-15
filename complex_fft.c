/*
 *  Copyright (c) 2011 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */


/*
 * This file contains the function ComplexFFT().
 * The description header can be found in signal_processing_library.h
 *
 */

#include "complex_fft_tables.h"
#include "signal_processing_library.h"

#define CFFTSFT 14
#define CFFTRND 1
#define CFFTRND2 16384

#define CIFFTSFT 14
#define CIFFTRND 1


int ComplexFFT(int16_t frfi[], int stages, int mode)
{
    /* The 1024-value is a constant given from the size of kSinTable1024[],
     * and should not be changed depending on the input parameter 'stages'
     */
    const int n = 1 << stages;
    if (n > 1024)
        return -1;

    int l = 1;
    int k = 10 - 1; /* Constant for given kSinTable1024[]. Do not change
         depending on the input parameter 'stages' */

    // 高精度モードのみを残す
    while (l < n)
    {
        int istep = l << 1;

        for (int m = 0; m < l; ++m)
        {
            int j = m << k;

            /* The 256-value is a constant given as 1/4 of the size of
             * kSinTable1024[], and should not be changed depending on the input
             * parameter 'stages'. It will result in 0 <= j < N_SINE_WAVE/2
             */
            int16_t wr = kSinTable1024[j + 256];
            int16_t wi = -kSinTable1024[j];

            for (int i = m; i < n; i += istep)
            {
                j = i + l;
                
                int32_t tr32 = wr * frfi[2 * j] - wi * frfi[2 * j + 1] + CFFTRND;

                int32_t ti32 = wr * frfi[2 * j + 1] + wi * frfi[2 * j] + CFFTRND;

                tr32 >>= 15 - CFFTSFT;
                ti32 >>= 15 - CFFTSFT;

                int32_t qr32 = ((int32_t)frfi[2 * i]) * (1 << CFFTSFT);
                int32_t qi32 = ((int32_t)frfi[2 * i + 1]) * (1 << CFFTSFT);

                frfi[2 * j] = (int16_t)(
                    (qr32 - tr32 + CFFTRND2) >> (1 + CFFTSFT));
                frfi[2 * j + 1] = (int16_t)(
                    (qi32 - ti32 + CFFTRND2) >> (1 + CFFTSFT));
                frfi[2 * i] = (int16_t)(
                    (qr32 + tr32 + CFFTRND2) >> (1 + CFFTSFT));
                frfi[2 * i + 1] = (int16_t)(
                    (qi32 + ti32 + CFFTRND2) >> (1 + CFFTSFT));
            }
        }

        --k;
        l = istep;
    }
    return 0;
}

int ComplexIFFT(int16_t frfi[], int stages, int mode)
{
    /* The 1024-value is a constant given from the size of kSinTable1024[],
     * and should not be changed depending on the input parameter 'stages'
     */
    const size_t n = ((size_t)1) << stages;
    if (n > 1024)
        return -1;

    int scale = 0;

    size_t l = 1;
    int k = 10 - 1; /* Constant for given kSinTable1024[]. Do not change
         depending on the input parameter 'stages' */

    while (l < n)
    {
        // variable scaling, depending upon data
        int shift = 0;
        int32_t round2 = 8192;

        int32_t tmp32 = MaxAbsValueW16(frfi, 2 * n);
        if (tmp32 > 13573)
        {
            shift++;
            scale++;
            round2 <<= 1;
        }
        if (tmp32 > 27146)
        {
            shift++;
            scale++;
            round2 <<= 1;
        }

        size_t istep = l << 1;

        // 高精度モードのみを残す
        for (size_t m = 0; m < l; ++m)
        {
            size_t j = m << k;

            /* The 256-value is a constant given as 1/4 of the size of
             * kSinTable1024[], and should not be changed depending on the input
             * parameter 'stages'. It will result in 0 <= j < N_SINE_WAVE/2
             */
            int16_t wr = kSinTable1024[j + 256];
            int16_t wi = kSinTable1024[j];

            for (size_t i = m; i < n; i += istep)
            {
                j = i + l;
                
                int32_t tr32 = wr * frfi[2 * j] - wi * frfi[2 * j + 1] + CIFFTRND;

                int32_t ti32 = wr * frfi[2 * j + 1] + wi * frfi[2 * j] + CIFFTRND;
                tr32 >>= 15 - CIFFTSFT;
                ti32 >>= 15 - CIFFTSFT;

                int32_t qr32 = ((int32_t)frfi[2 * i]) * (1 << CIFFTSFT);
                int32_t qi32 = ((int32_t)frfi[2 * i + 1]) * (1 << CIFFTSFT);

                frfi[2 * j] = (int16_t)(
                    (qr32 - tr32 + round2) >> (shift + CIFFTSFT));
                frfi[2 * j + 1] = (int16_t)(
                    (qi32 - ti32 + round2) >> (shift + CIFFTSFT));
                frfi[2 * i] = (int16_t)(
                    (qr32 + tr32 + round2) >> (shift + CIFFTSFT));
                frfi[2 * i + 1] = (int16_t)(
                    (qi32 + ti32 + round2) >> (shift + CIFFTSFT));
            }
        }
        --k;
        l = istep;
    }
    return scale;
}
