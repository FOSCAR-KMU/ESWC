#ifndef V4L2_H_
#define V4L2_H_

#include "util.h"
#include <linux/videodev2.h>

struct v4l2 {
    int fd;
    int nbufs;
    struct v4l2_buffer *v4l2bufs;
    struct buffer **bufs;
};

/* Print v4l2 related help */
void v4l2_usage(void);

/* Open v4l2  */
struct v4l2 * v4l2_open(uint32_t fourcc, uint32_t width, uint32_t height);
void v4l2_close(struct v4l2 * v4l2);

/* Share the buffers w/ v4l2 via dmabuf */
int v4l2_reqbufs(struct v4l2 *v4l2, uint32_t n);

int v4l2_streamon(struct v4l2 *v4l2);
int v4l2_streamoff(struct v4l2 *v4l2);

/* Queue a buffer to the camera */
int v4l2_qbuf(struct v4l2 *v4l2, int dmafd, int index);

/* Dequeue buffer from camera */
int v4l2_dqbuf(struct v4l2 *v4l2, int* field);

#endif
