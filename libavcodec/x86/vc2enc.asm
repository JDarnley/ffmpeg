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

SECTION .text

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

INIT_XMM sse2
HAAR_BLOCK

INIT_XMM avx
HAAR_BLOCK

INIT_YMM avx2
HAAR_BLOCK
