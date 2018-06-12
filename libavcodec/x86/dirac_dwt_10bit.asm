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
pd_2: times 4 dd 2
pd_8: times 4 dd 8

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

%macro HAAR_HORIZONTAL 0

cglobal horizontal_compose_haar_10bit, 3, 6, 4, b, temp_, w, x, b2
    mova m2, [pd_1]
    xor xd, xd
    shr wd, 1
    lea b2q, [bq + 4*wq]

    ALIGN 16
    .loop_lo:
        mova m0, [bq  + 4*xq]
        movu m1, [b2q + 4*xq]
        paddd m1, m2
        psrad m1, 1
        psubd m0, m1
        mova [temp_q + 4*xq], m0
        add xd, mmsize/4
        cmp xd, wd
    jl .loop_lo

    xor xd, xd
    and wd, ~(mmsize/4 - 1)
    cmp wd, mmsize/4
    jl .end

    ALIGN 16
    .loop_hi:
        mova m0, [temp_q + 4*xq]
        movu m1, [b2q    + 4*xq]
        paddd m1, m0
        paddd m0, m2
        paddd m1, m2
        psrad m0, 1
        psrad m1, 1
        SBUTTERFLY dq, 0,1,3
        mova [bq + 8*xq], m0
        mova [bq + 8*xq + mmsize], m1
        add xd, mmsize/4
        cmp xd, wd
    jl .loop_hi
    .end:
REP_RET

%endmacro

%macro LEGALL53_VERTICAL_LO 0

cglobal legall53_vertical_lo, 4, 4, 4, b0, b1, b2, w
    mova m3, [pd_2]
    shl wd, 2
    add b0q, wq
    add b1q, wq
    add b2q, wq
    neg wq

    ALIGN 16
    .loop:
        mova m0, [b0q + wq]
        mova m1, [b1q + wq]
        mova m2, [b2q + wq]
        paddd m0, m2
        paddd m0, m3
        psrad m0, 2
        psubd m1, m0
        mova [b1q + wq], m1
        add wq, mmsize
    jl .loop
RET

%endmacro

%macro LEGALL53_VERTICAL_HI 0

cglobal legall53_vertical_hi, 4, 4, 4, b0, b1, b2, w
    mova m3, [pd_1]
    shl wd, 2
    add b0q, wq
    add b1q, wq
    add b2q, wq
    neg wq

    ALIGN 16
    .loop:
        mova m0, [b0q + wq]
        mova m1, [b1q + wq]
        mova m2, [b2q + wq]
        paddd m0, m2
        paddd m0, m3
        psrad m0, 1
        paddd m1, m0
        mova [b1q + wq], m1
        add wq, mmsize
    jl .loop
RET

%endmacro

%macro DD97_VERTICAL_HI 0

cglobal dd97_vertical_hi, 6, 6, 11, b0, b1, b2, b3, b4, w
    mova m10, [pd_8]
    shl wd, 2
    add b0q, wq
    add b1q, wq
    add b2q, wq
    add b3q, wq
    add b4q, wq
    neg wq

    ALIGN 16
    .loop:
        mova m0, [b0q + wq]
        mova m1, [b1q + wq]
        mova m2, [b2q + wq]
        mova m3, [b3q + wq]
        mova m4, [b4q + wq]
        pslld m5, m1, 3
        pslld m6, m3, 3
        paddd m5, m1
        paddd m6, m3
        psubd m5, m0
        psubd m6, m4
        paddd m5, m10
        paddd m5, m6
        psrad m5, 4
        paddd m2, m5
        mova [b2q + wq], m2
        add wq, mmsize
    jl .loop
RET

%endmacro

INIT_XMM sse2
DD97_VERTICAL_HI
HAAR_HORIZONTAL
HAAR_VERTICAL
LEGALL53_VERTICAL_HI
LEGALL53_VERTICAL_LO

INIT_XMM avx
HAAR_HORIZONTAL
HAAR_VERTICAL
