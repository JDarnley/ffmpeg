;******************************************************************************
;* Copyright (c) 2017 James Darnley <jdarnley@obe.tv>
;*
;* This file is part of FFmpeg.
;*
;* FFmpeg is free software; you can redistribute it and/or
;* modify it under the terms of the GNU Lesser General Public
;* License as published by the Free Software Foundation; either
;* version 2.1 of the License, or (at your option) any later version.
;*
;* FFmpeg is distributed in the hope that it will be useful,
;* but WITHOUT ANY WARRANTY; without even the implied warranty of
;* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;* Lesser General Public License for more details.
;*
;* You should have received a copy of the GNU Lesser General Public
;* License along with FFmpeg; if not, write to the Free Software
;* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
;******************************************************************************

%include "libavutil/x86/x86util.asm"

SECTION_RODATA 32

cextern pd_1

deint_even: db 0, 1, 2, 3,  8,  9, 10, 11
            times 8 db 0
deint_odd:  db 4, 5, 6, 7, 12, 13, 14, 15
            times 8 db 0

SECTION .text

INIT_XMM avx

cglobal vc2enc_dwt_haar, 5, 7, 16, dst, src, stride, w, h, x, y, orig_dst
    shl wd, 1
    shl hd, 1

    mov orig_dstq, dstq

    ; horizontal synthesis (hs)
    xor yq, yq
    .hs_loop_height:
        xor xq, xq
        .hs_loop_width:
            movu m2, [srcq + 4*xq]

            pshufb m0, m2, [deint_even]
            pshufb m1, m2, [deint_odd]

            pslld m0, 1
            pslld m1, 1

            psubd m2, m1, m0
            paddd m3, m2, [pd_1]
            psrad m3, 1
            paddd m0, m3

            punpckldq m0, m2

            movu [dstq + 4*xq], m0
            add xq, mmsize/4
            cmp xq, wq
        jl .hs_loop_width

        lea srcq, [srcq + 4*strideq]
        lea dstq, [dstq + 4*wq]
        add yq, 1
        cmp yq, hq
    jl .hs_loop_height

    ; vertical synthesis (vs)
    xor xq, xq
    .vs_loop_width:
        lea dstq, [orig_dstq + 4*xq]
        xor yq, yq
        .vs_loop_height:
            movu m0, [dstq]
            movu m1, [dstq + 4*wq]

            psubd m2, m1, m0
            paddd m3, m2, [pd_1]
            psrad m3, 1
            paddd m0, m3

            movu [dstq], m0
            movu [dstq + 4*wq], m2
            lea dstq, [dstq + 8*wq]
            add yq, 2
            cmp yq, hq
        jl .vs_loop_height

        add xq, mmsize/4
        cmp xq, wq
    jl .vs_loop_width
RET
