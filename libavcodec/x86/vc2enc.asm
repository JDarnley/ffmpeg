;******************************************************************************
;* Copyright (c) 2018 James Darnley <jdarnley@obe.tv>
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

SECTION_RODATA 16

cextern pd_1
pd_2: times 4 dd 2

SECTION .text

%macro LOAD_PIXEL_DATA 0

cglobal load_pixel_data, 6, 7, 2 + notcpuflag(sse4), pixels_, coeffs_, pixel_stride_, coeff_stride_, w, h
    shl coeff_stride_q, 2
    VPBROADCASTD m1, r7m ; diff_offset
    movsxd wq, wd
    lea coeffs_q, [coeffs_q + 4*wq]
    %if notcpuflag(sse4)
        pxor m2, m2
    %endif

    cmp dword r6m, 2 ; bytes_per_pixel
    je .two

    add pixels_q, wq
    neg wq
    mov r6, wq
    ALIGN 16
    .one_loop_h:
        .one_loop_w:
            %if cpuflag(sse4)
                pmovzxbd m0, [pixels_q + wq]
            %else
                movd m0, [pixels_q + wq]
                punpcklbw m0, m2
                punpcklwd m0, m2
            %endif
            psubd m0, m1
            mova [coeffs_q + 4*wq], m0

            add wq, mmsize/4
        jl .one_loop_w

        mov wq, r6
        add pixels_q, pixel_stride_q
        add coeffs_q, coeff_stride_q
        sub hd, 1
    jg .one_loop_h
    RET

    .two:
    lea pixels_q, [pixels_q + 2*wq]
    neg wq
    mov r6, wq
    ALIGN 16
    .two_loop_h:
        .two_loop_w:
            %if cpuflag(sse4)
                pmovzxwd m0, [pixels_q + 2*wq]
            %else
                movq m0, [pixels_q + 2*wq]
                punpcklwd m0, m2
            %endif
            psubd m0, m1
            mova [coeffs_q + 4*wq], m0

            add wq, mmsize/4
        jl .two_loop_w

        mov wq, r6
        add pixels_q, pixel_stride_q
        add coeffs_q, coeff_stride_q
        sub hd, 1
    jg .two_loop_h
RET

%endmacro

%macro HAAR_BLOCK 0

cglobal haar_block, 4, 6, 7, data_, stride_, w, h
    mov r4, data_q
    mov r5d, wd
    mova m6, [pd_1]

    ALIGN 16
    .loop_h:
        .loop_w:
            mova m0, [data_q]
            mova m1, [data_q + mmsize]
            mova m2, [data_q + 4*stride_q]
            mova m3, [data_q + 4*stride_q + mmsize]
            shufps m4, m0, m1, q3131
            shufps m0, m1, q2020
            shufps m5, m2, m3, q3131
            shufps m2, m3, q2020
            SWAP 1,4
            SWAP 3,5

            pslld m0, 1
            pslld m1, 1
            psubd m1, m0
            paddd m4, m1, m6
            psrad m4, 1
            paddd m0, m4
            SBUTTERFLY dq, 0, 1, 4

            pslld  m2, 1
            pslld  m3, 1
            psubd  m3, m2
            paddd  m5, m3, m6
            psrad  m5, 1
            paddd  m2, m5
            SBUTTERFLY dq, 2, 3, 5

            psubd m2, m0
            psubd m3, m1
            paddd m4, m2, m6
            paddd m5, m3, m6
            psrad m4, 1
            psrad m5, 1
            paddd m0, m4
            paddd m1, m5

            mova [data_q], m0
            mova [data_q + mmsize], m1
            mova [data_q + 4*stride_q], m2
            mova [data_q + 4*stride_q + mmsize], m3

            add data_q, 2*mmsize
            sub wd, 2*mmsize/4
        jnz .loop_w

        mov wd, r5d
        lea r4, [r4 + 2*4*stride_q]
        mov data_q, r4
        sub hd, 2
    jnz .loop_h
RET

%endmacro

%macro LEGALL_HFILTER_STAGE1 0

cglobal legall_hfilter_stage1, 2, 2, 6, data_, w
    mova m5, [pd_2]

    ALIGN 16
    .loop:
        movu m0, [data_q]
        movu m1, [data_q + mmsize]
        movu m2, [data_q - 4]
        movu m3, [data_q - 4 + mmsize]
        shufps m4, m0, m1, q3131
        shufps m0, m1, q2020
        shufps m2, m3, q2020
        SWAP 1,4

        pslld m0, 1
        paddd m2, m1
        paddd m2, m5
        psrad m2, 2
        paddd m0, m2
        SBUTTERFLY dq, 0, 1, 3

        movu [data_q], m0
        movu [data_q + mmsize], m1

        add data_q, 2*mmsize
        sub wd, mmsize/4
    jg .loop
RET

%endmacro

%macro LEGALL_HFILTER_STAGE2 0

cglobal legall_hfilter_stage2, 2, 2, 6, data_, w
    mova m5, [pd_1]

    ALIGN 16
    .loop:
        mova m0, [data_q]
        mova m1, [data_q + mmsize]
        movu m2, [data_q + 8]
        movu m3, [data_q + 8 + mmsize]
        shufps m4, m0, m1, q3131
        shufps m0, m1, q2020
        shufps m2, m3, q2020
        SWAP 1,4

        paddd m2, m0
        pslld m1, 1
        pslld m2, 1
        paddd m2, m5
        psrad m2, 1
        psubd m1, m2
        SBUTTERFLY dq, 0, 1, 3

        mova [data_q], m0
        mova [data_q + mmsize], m1

        add data_q, 2*mmsize
        sub wd, mmsize/4
    jg .loop
RET

%endmacro

%macro LEGALL_VFILTER_STAGE1 0

cglobal legall_vfilter_stage1, 4, 6, 4, data_, stride_, w, h
    mov r4, data_q
    mov r5d, wd
    mova m3, [pd_2]

    ALIGN 16
    .loop_h:
        .loop_w:
            mova m0, [data_q]
            mova m1, [data_q + 4*stride_q]
            mova m2, [data_q + 8*stride_q]

            paddd m0, m2
            paddd m0, m3
            psrad m0, 2
            paddd m1, m0

            mova [data_q + 4*stride_q], m1
            add data_q, mmsize
            sub wd, mmsize/4
        jg .loop_w

        mov wd, r5d
        lea r4, [r4 + 8*stride_q]
        mov data_q, r4
        sub hd, 2
    jg .loop_h
RET

%endmacro

%macro LEGALL_VFILTER_STAGE2 0

cglobal legall_vfilter_stage2, 4, 6, 4, data_, stride_, w, h
    mov r4, data_q
    mov r5d, wd
    mova m3, [pd_1]

    ALIGN 16
    .loop_h:
        .loop_w:
            mova m0, [data_q]
            mova m1, [data_q + 4*stride_q]
            mova m2, [data_q + 8*stride_q]

            paddd m0, m2
            paddd m0, m3
            psrad m0, 1
            psubd m1, m0

            mova [data_q + 4*stride_q], m1
            add data_q, mmsize
            sub wd, mmsize/4
        jg .loop_w

        mov wd, r5d
        lea r4, [r4 + 8*stride_q]
        mov data_q, r4
        sub hd, 2
    jg .loop_h
RET

%endmacro

INIT_XMM sse2
HAAR_BLOCK
LEGALL_HFILTER_STAGE1
LEGALL_HFILTER_STAGE2
LEGALL_VFILTER_STAGE1
LEGALL_VFILTER_STAGE2
LOAD_PIXEL_DATA

INIT_XMM sse4
LOAD_PIXEL_DATA

INIT_XMM avx
HAAR_BLOCK
LEGALL_HFILTER_STAGE1
LEGALL_HFILTER_STAGE2

INIT_YMM avx2
HAAR_BLOCK
LOAD_PIXEL_DATA
