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

/**
 * @file
 * video decoding with libavcodec API example
 *
 * @example decode_video.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libavutil/pixdesc.h>
#include <libavcodec/avcodec.h>

static void save_plane(uint8_t *data, ptrdiff_t stride, int width, int height, FILE *file)
{
    for (int y = 0; y < height; y++) {
        fwrite(data + y*stride, 1, width, file);
    }
}

int main(int argc, char **argv)
{
    const char *filename, *outfilename;
    const AVCodec *codec;
    AVCodecParserContext *parser;
    AVCodecContext *c= NULL;
    FILE *file_in, *file_out;
    AVFrame *frame;
    uint8_t *inbuf = NULL;
    uint8_t *data;
    size_t   data_size;
    int ret;
    AVPacket *pkt;

    if (argc != 3) {
        fprintf(stderr, "Usage: %s <input file> <output file>\n", argv[0]);
        exit(0);
    }
    filename    = argv[1];
    outfilename = argv[2];

    av_log_set_level(AV_LOG_DEBUG);

    avcodec_register_all();

    pkt = av_packet_alloc();
    if (!pkt)
        exit(1);

    /* find the MPEG-1 video decoder */
    codec = avcodec_find_decoder(AV_CODEC_ID_DIRAC);
    if (!codec) {
        fprintf(stderr, "Codec not found\n");
        exit(1);
    }

    c = avcodec_alloc_context3(codec);
    if (!c) {
        fprintf(stderr, "Could not allocate video codec context\n");
        exit(1);
    }

    /* For some codecs, such as msmpeg4 and mpeg4, width and height
       MUST be initialized there because this information is not
       available in the bitstream. */

    /* open it */
    if (avcodec_open2(c, codec, NULL) < 0) {
        fprintf(stderr, "Could not open codec\n");
        exit(1);
    }

    file_in = fopen(filename, "rb");
    if (!file_in) {
        fprintf(stderr, "Could not open %s\n", filename);
        exit(1);
    }

    file_out = fopen(outfilename, "wb");
    if (!file_out) {
        fprintf(stderr, "Could not open %s\n", outfilename);
        exit(1);
    }

    if (fseek(file_in, 0, SEEK_END))
        exit(1);
    size_t file_size = ftell(file_in);
    if (fseek(file_in, 0, SEEK_SET))
        exit(1);

    inbuf = malloc(file_size + AV_INPUT_BUFFER_PADDING_SIZE);
    if (!inbuf)
        exit(1);

    ret = fread(inbuf, 1, file_size, file_in);
    if (ret != file_size)
        exit(1);

    fclose(file_in);

    /* set end of buffer to 0 (this ensures that no overreading happens for damaged MPEG streams) */
    memset(inbuf + file_size, 0, AV_INPUT_BUFFER_PADDING_SIZE);

    frame = av_frame_alloc();
    if (!frame) {
        fprintf(stderr, "Could not allocate video frame\n");
        exit(1);
    }

    pkt->data = inbuf;
    pkt->size = file_size;
    //decode(c, frame, pkt, outfilename);

    ret = avcodec_send_packet(c, pkt);
    if (ret < 0) {
        fprintf(stderr, "Error sending a packet for decoding\n");
        exit(1);
    }

    ret = avcodec_receive_frame(c, frame);
    if (ret) {
        fprintf(stderr, "Error during decoding\n");
        exit(1);
    }

    fprintf(stderr, "Received '%s' format AVFrame (%dx%d)\n",
            av_get_pix_fmt_name(frame->format), frame->width, frame->height);
    AVPixFmtDescriptor *pix_fmt_desc = av_pix_fmt_desc_get(frame->format);

    for (int i = 0; i < 3; i++) {
        int w = frame->width, h = frame->height;
        if (i) {
            w >>= pix_fmt_desc->log2_chroma_w;
            h >>= pix_fmt_desc->log2_chroma_h;
        }
        save_plane(frame->data[i], frame->linesize[i],
                w*pix_fmt_desc->comp[i].step, h, file_out);
    }

    fclose(file_out);
    avcodec_free_context(&c);
    av_frame_free(&frame);
    av_packet_free(&pkt);

    return 0;
}
