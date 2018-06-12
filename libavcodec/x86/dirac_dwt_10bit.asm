;******************************************************************************
;* x86 optimized discrete 10-bit wavelet trasnform
;* Copyright (c) 2018 James Darnley
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
;* 51, Inc., Foundation Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
;******************************************************************************

%include "libavutil/x86/x86util.asm"

SECTION_RODATA

cextern pd_1

SECTION .text

%macro HAAR_VERTICAL 0

cglobal vertical_compose_haar_10bit, 3, 3, 4, b0, b1, w
    mova m2, [pd_1]
    shl wd, 2
    add b0q, wq
    add b1q, wq
    neg wq

    ALIGN 16
    .loop:
        mova m0, [b0q + wq]
        mova m1, [b1q + wq]
        paddd m3, m1, m2
        psrad m3, 1
        psubd m0, m3
        paddd m1, m0
        mova [b0q + wq], m0
        mova [b1q + wq], m1
        add wq, mmsize
    jl .loop
RET

%endmacro

INIT_XMM sse2
HAAR_VERTICAL

INIT_XMM avx
HAAR_VERTICAL
