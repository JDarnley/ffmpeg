/*
 * Copyright (c) 2001 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#define MAX_SLICES 8
#define MAX_FRAMES 300

#include "config.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if HAVE_UNISTD_H
#include <unistd.h>
#endif
#if HAVE_IO_H
#include <io.h>
#endif
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <pthread.h>

#include "libavcodec/avcodec.h"
#include "libavutil/pixdesc.h"
#include "libavutil/hash.h"
#include "libavutil/bswap.h"
#include "libavutil/imgutils.h"

/* TODO: drop back to 1 frame buffer. */
uint8_t *slice_byte_buffer[MAX_FRAMES];
int slice_byte_buffer_size;
int draw_horiz_band_called;

pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;

FILE *fh_frame, *fh_slice;

static void draw_horiz_band(AVCodecContext *ctx, const AVFrame *fr, int offset[4],
                            int slice_position, int type, int height)
{
    int i;
    const AVPixFmtDescriptor *pix_fmt_desc;
    int chroma_w, chroma_h;
    int shift_slice_position;
    int shift_height;
    uint8_t *buffer = slice_byte_buffer[fr->coded_picture_number];

   //pthread_mutex_lock(&mutex1);
#if 0
    fprintf(stderr, "draw_horiz_band, AVFrame: %p, coded_picture_number: %d, " /*offset: %d, %d, %d, %d, linesize %d, %d, %d, %d*/ " y, %d, type: %d, height: %d\n",
            fr->data[0], fr->coded_picture_number,
            //offset[0], offset[1], offset[2], offset[3],
            //fr->linesize[0], fr->linesize[1], fr->linesize[2], fr->linesize[3],
            slice_position,
            type, height);
#endif

    draw_horiz_band_called = 1;

    pix_fmt_desc = av_pix_fmt_desc_get(ctx->pix_fmt);
    chroma_w = -((-ctx->width) >> pix_fmt_desc->log2_chroma_w);
    chroma_h = -((-height) >> pix_fmt_desc->log2_chroma_h);
    shift_slice_position = -((-slice_position) >> pix_fmt_desc->log2_chroma_h);
    shift_height = -((-ctx->height) >> pix_fmt_desc->log2_chroma_h);

    assert(fr->coded_picture_number < MAX_FRAMES);

    for (i = 0; i < height; i++) {
        memcpy(buffer + ctx->width * slice_position + i * ctx->width,
               fr->data[0] + offset[0] + i * fr->linesize[0], ctx->width);
    }

    for (i = 0; i < chroma_h; i++) {
        memcpy(buffer + ctx->width * ctx->height + chroma_w * shift_slice_position + i * chroma_w,
               fr->data[1] + offset[1] + i * fr->linesize[1], chroma_w);
    }
    for (i = 0; i < chroma_h; i++) {
        memcpy(buffer + ctx->width * ctx->height + chroma_w * shift_height + chroma_w * shift_slice_position + i * chroma_w,
               fr->data[2] + offset[2] + i * fr->linesize[2], chroma_w);
    }

    //pthread_mutex_unlock( &mutex1 );
}

static int identical = 1;

static int decode(AVCodecContext *dec_ctx, AVFrame *frame,
           AVPacket *pkt)
{
    static uint64_t frame_cnt = 0;
    int ret;

    ret = avcodec_send_packet(dec_ctx, pkt);
    if (ret < 0) {
        fprintf(stderr, "Error sending a packet for decoding: %s\n", av_err2str(ret));
        return ret;
    }

    while (ret >= 0) {
        int chroma_w, chroma_h;
        const AVPixFmtDescriptor *desc;
        char sum[AV_HASH_MAX_SIZE * 2 + 1];
        char sum_slice[AV_HASH_MAX_SIZE * 2 + 1];
        struct AVHashContext *hash;
        uint8_t *buffer;
        int frame_size;

        ret = avcodec_receive_frame(dec_ctx, frame);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            return 0;
        } else if (ret < 0) {
            fprintf(stderr, "Error during decoding: %s\n", av_err2str(ret));
            return ret;
        }

        desc = av_pix_fmt_desc_get(dec_ctx->pix_fmt);
        if ((ret = av_hash_alloc(&hash, "md5")) < 0) {
            return ret;
        }
        av_hash_init(hash);

        chroma_w = -((-dec_ctx->width) >> desc->log2_chroma_w);
        chroma_h = -((-dec_ctx->height) >> desc->log2_chroma_h);

        for (int i = 0; i < frame->height; i++) {
            av_hash_update(hash, &frame->data[0][i * frame->linesize[0]], frame->width);
            if (fh_frame)
                fwrite(&frame->data[0][i * frame->linesize[0]], 1, frame->width, fh_frame);
        }
        for (int i = 0; i < frame->height >> desc->log2_chroma_h; i++) {
            av_hash_update(hash, &frame->data[1][i * frame->linesize[1]], frame->width >> desc->log2_chroma_w);
            if (fh_frame)
                fwrite(&frame->data[1][i * frame->linesize[1]], 1, chroma_w, fh_frame);
        }
        for (int i = 0; i < frame->height >> desc->log2_chroma_h; i++) {
            av_hash_update(hash, &frame->data[2][i * frame->linesize[2]], frame->width >> desc->log2_chroma_w);
            if (fh_frame)
                fwrite(&frame->data[2][i * frame->linesize[2]], 1, chroma_w, fh_frame);
        }

        av_hash_final_hex(hash, sum, av_hash_get_size(hash) * 2 + 1);

        assert(frame_cnt < MAX_FRAMES);
        buffer = slice_byte_buffer[frame_cnt];
        frame_size = dec_ctx->width*dec_ctx->height + 2*chroma_w*chroma_h;
        av_hash_init(hash);
        av_hash_update(hash, buffer, frame_size);
        av_hash_final_hex(hash, sum_slice, av_hash_get_size(hash) * 2 + 1);

        if (fh_slice)
            fwrite(buffer, 1, frame_size, fh_slice);

        if (strcmp(sum, sum_slice)) {
            fprintf(stderr, "frame %"PRIu64" not identical\n", frame_cnt);
            identical = 0;
        }

        frame_cnt += 1;
        av_hash_freep(&hash);
        memset(buffer, 0, slice_byte_buffer_size);
    }
    return 0;
}

int main(int argc, char **argv)
{
    const AVCodec *codec = NULL;
    AVCodecContext *c = NULL;
    AVFrame *frame = NULL;
    unsigned int threads;
    AVPacket *pkt;
    FILE *file = NULL;
    char * nal = NULL;
    int nals = 0, ret = 0;
    char *p;

    if (argc < 3) {
        fprintf(stderr, "Usage: %s <threads> <input file> [<raw_frame_output> <raw_slice_output>]\n", argv[0]);
        return -1;
    }

    if (!(threads = strtoul(argv[1], NULL, 0)))
        threads = 1;
    else if (threads > MAX_SLICES)
        threads = MAX_SLICES;

#ifdef _WIN32
    setmode(fileno(stdout), O_BINARY);
#endif

    if (!(pkt = av_packet_alloc())) {
        return -1;
    }

    if (argc >= 5) {
        fh_frame = fopen(argv[3], "wb");
        if (!fh_frame) {
            perror(argv[3]);
            goto err;
        }

        fh_slice = fopen(argv[4], "wb");
        if (!fh_slice) {
            perror(argv[4]);
            goto err;
        }
    }

    nal = av_malloc(MAX_SLICES * UINT16_MAX + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!nal)
        goto err;
    p = nal;

    if (!(codec = avcodec_find_decoder(AV_CODEC_ID_H264))) {
        fprintf(stderr, "Codec not found\n");
        ret = -1;
        goto err;
    }

    if (!(c = avcodec_alloc_context3(codec))) {
        fprintf(stderr, "Could not allocate video codec context\n");
        ret = -1;
        goto err;
    }

    c->width  = 704;
    c->height = 576;

    c->flags2 |= AV_CODEC_FLAG2_CHUNKS;
    c->thread_type = FF_THREAD_SLICE;
    c->thread_count = threads;
    c->draw_horiz_band = draw_horiz_band;
    c->slice_flags = SLICE_FLAG_ALLOW_FIELD;

    if ((ret = avcodec_open2(c, codec, NULL)) < 0) {
        fprintf(stderr, "Could not open codec\n");
        goto err;
    }

    slice_byte_buffer_size = av_image_get_buffer_size(AV_PIX_FMT_YUV420P, c->width, c->height, 32);
    assert(slice_byte_buffer_size > 0);
    for (int i = 0; i < MAX_FRAMES; i++) {
        slice_byte_buffer[i] = av_mallocz(slice_byte_buffer_size);
        assert(slice_byte_buffer[i]);
    }

#if HAVE_THREADS
    if (c->active_thread_type == FF_THREAD_FRAME) {
        fprintf(stderr, "Couldn't activate slice threading: %d\n", c->active_thread_type);
        ret = -1;
        goto err;
    }
#else
    fprintf(stderr, "WARN: not using threads, only checking decoding slice NALUs\n");
#endif

    if (!(frame = av_frame_alloc())) {
        fprintf(stderr, "Could not allocate video frame\n");
        ret = -1;
        goto err;
    }

    if (!(file = fopen(argv[2], "rb"))) {
        fprintf(stderr, "Couldn't open NALU file: %s\n", argv[2]);
        ret = -1;
        goto err;
    }

    //FILE *test = fopen("raw.h264", "wb");

    while(1) {
        uint16_t size = 0;
        size_t ret = fread(&size, 1, sizeof(uint16_t), file);
        if (ret != sizeof(uint16_t))
            break;

        size = av_be2ne16(size);
        ret = fread(p, 1, size, file);
        if (ret != size) {
            perror("Couldn't read data");
            goto err;
        }
        p += ret;

        if (++nals >= threads) {
            int decret = 0;
            pkt->data = nal;
            pkt->size = p - nal;
            //fwrite(pkt->data, 1, pkt->size, test);
            if ((decret = decode(c, frame, pkt)) < 0) {
                goto err;
            }
            memset(nal, 0, MAX_SLICES * UINT16_MAX + AV_INPUT_BUFFER_PADDING_SIZE);
            nals = 0;
            p = nal;
        }
    }

    if (nals) {
        pkt->data = nal;
        pkt->size = p - nal;
        if ((ret = decode(c, frame, pkt)) < 0) {
            goto err;
        }
    }

    ret = decode(c, frame, NULL);

err:
    if (nal)
        av_free(nal);
    if (file)
        fclose(file);
    av_frame_free(&frame);
    avcodec_free_context(&c);
    av_packet_free(&pkt);
    for (int i = 0; i < MAX_FRAMES; i++)
        av_freep(&slice_byte_buffer[i]);

    if (fh_frame)
        fclose(fh_frame);
    if (fh_slice)
        fclose(fh_slice);

    if (!identical)
        return 1;
    else
        return ret;
}
