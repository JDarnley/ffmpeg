#include "libavutil/cpu.h"
#include "libavutil/x86/cpu.h"
#include "libavcodec/vc2enc_dwt.h"

void ff_haar_block_sse2(dwtcoef* data, ptrdiff_t stride, int width, int height);
void ff_haar_block_avx(dwtcoef* data, ptrdiff_t stride, int width, int height);
void ff_haar_block_avx2(dwtcoef* data, ptrdiff_t stride, int width, int height);

void ff_legall_vfilter_stage1_sse2(dwtcoef* data, ptrdiff_t stride, int width, int height);

av_cold void ff_vc2enc_init_transforms_x86(VC2TransformContext *s)
{
    int cpuflags = av_get_cpu_flags();

    if (EXTERNAL_SSE2(cpuflags)) {
        s->alignment = 2 * 16 / sizeof (dwtcoef);
        s->haar_block = ff_haar_block_sse2;
        s->legall_vfilter_stage1 = ff_legall_vfilter_stage1_sse2;
    }

    if (EXTERNAL_AVX(cpuflags)) {
        s->alignment = 2 * 16 / sizeof (dwtcoef);
        s->haar_block = ff_haar_block_avx;
    }

    if (EXTERNAL_AVX2(cpuflags)) {
        s->alignment = 2 * 32 / sizeof (dwtcoef);
        s->haar_block = ff_haar_block_avx2;
    }
}
