#include "libavutil/attributes.h"
#include "libavutil/cpu.h"
#include "libavutil/x86/cpu.h"
#include "libavcodec/vc2enc_dwt.h"

void ff_vc2enc_dwt_haar_avx(dwtcoef* output, dwtcoef *input, ptrdiff_t stride, int width, int height);

static av_always_inline void deinterleave(dwtcoef *linell, ptrdiff_t stride,
                                          int width, int height, dwtcoef *synthl)
{
    int x, y;
    ptrdiff_t synthw = width << 1;
    dwtcoef *linehl = linell + width;
    dwtcoef *linelh = linell + height*stride;
    dwtcoef *linehh = linelh + width;

    /* Deinterleave the coefficients. */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            linell[x] = synthl[(x << 1)];
            linehl[x] = synthl[(x << 1) + 1];
            linelh[x] = synthl[(x << 1) + synthw];
            linehh[x] = synthl[(x << 1) + synthw + 1];
        }
        synthl += synthw << 1;
        linell += stride;
        linelh += stride;
        linehl += stride;
        linehh += stride;
    }
}

static void haar_wrap(dwtcoef *synth, dwtcoef *input,
                      ptrdiff_t stride, int width, int height)
{
    ff_vc2enc_dwt_haar_avx(synth, input, stride, width, height);
    deinterleave(input, stride, width, height, synth);
}

av_cold void ff_vc2enc_init_transforms_x86(VC2TransformContext *s)
{
    int cpuflags = av_get_cpu_flags();

    if (EXTERNAL_AVX(cpuflags)) {
        s->vc2_subband_dwt[VC2_TRANSFORM_HAAR_S] = haar_wrap;
    }
}
