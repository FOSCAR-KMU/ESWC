#ifndef VPE_COMMON_H_
#define VPE_COMMON_H_

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>

#include "display-kms.h"
#include "util.h"

#define NUMBUF                          6

struct image_params {
    int width;
    int height;
    int fourcc;
    int size;
    int size_uv;
    int coplanar;
    enum v4l2_colorspace colorspace;
    int numbuf;
};

struct vpe {
    int fd;
    int field;
    int deint;
    int translen;
    struct image_params src;
    struct image_params dst;
    struct v4l2_crop crop;
    int input_buf_dmafd[NUMBUF];
    int input_buf_dmafd_uv[NUMBUF];
    int output_buf_dmafd[NUMBUF];
    int output_buf_dmafd_uv[NUMBUF];
    struct display *disp;
    struct buffer **disp_bufs;
};

struct vpe *vpe_open(void);
int vpe_close(struct vpe *vpe);
int describeFormat (char *format, struct image_params *image);
int vpe_input_init(struct vpe *vpe);
int vpe_output_init(struct vpe *vpe);
int vpe_input_qbuf(struct vpe *vpe, int index);
int vpe_output_qbuf(struct vpe *vpe, int index);
int vpe_stream_on(int fd, int type);
int vpe_stream_off(int fd, int type);
int vpe_input_dqbuf(struct vpe *vpe);
int vpe_output_dqbuf(struct vpe *vpe);
void vpe_output_fullscreen(struct vpe* vpe, bool bfull);

#endif // VPE_COMMON_H_

