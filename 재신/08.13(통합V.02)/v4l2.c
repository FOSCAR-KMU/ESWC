/*
 * Copyright (C) 2011 Texas Instruments
 * Author: Rob Clark <rob.clark@linaro.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//#ifdef HAVE_CONFIG_H
#include "config.h"
//#endif

#include <linux/videodev2.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>

#include <omap_drmif.h>

#include "util.h"
#include "v4l2.h"

void
v4l2_usage(void)
{
    MSG("V4L2 Capture Options:");
    MSG("\t-c WxH@fourcc\tset capture dimensions/format");
    MSG("\t-m\t\tdo MCF setup");
}

struct v4l2* v4l2_open(uint32_t fourcc, uint32_t width, uint32_t height)
{
    struct v4l2_format format = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    };
    struct v4l2 *v4l2;
    int ret;
    
    char devname[20] = "/dev/video1";

    v4l2 = calloc(1, sizeof(*v4l2));

    v4l2->fd = open(devname, O_RDWR);

    ret = ioctl(v4l2->fd, VIDIOC_G_FMT, &format);
    if (ret < 0) {
        ERROR("VIDIOC_G_FMT failed: %s (%d)", strerror(errno), ret);
        goto fail;
    }

    format.fmt.pix.pixelformat = fourcc;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;

    if ((format.fmt.pix.width == 0) || (format.fmt.pix.height == 0) ||
            (format.fmt.pix.pixelformat == 0)) {

        ERROR("invalid capture settings '%dx%d@%4s'",
            format.fmt.pix.width, format.fmt.pix.height,
            (char *)&format.fmt.pix.pixelformat);
        goto fail;
    }

    ret = ioctl(v4l2->fd, VIDIOC_S_FMT, &format);
    if (ret < 0) {
        ERROR("VIDIOC_S_FMT failed: %s (%d)", strerror(errno), ret);
        goto fail;
    }

    return v4l2;

fail:
    // XXX cleanup
    free(v4l2);
    return NULL;
}

void v4l2_close(struct v4l2 * v4l2)
{
    close(v4l2->fd);
    free(v4l2);
}

int
v4l2_reqbufs(struct v4l2 *v4l2, uint32_t n)
{
#if 1
    int ret;
    struct v4l2_requestbuffers rqbufs;

    memset(&rqbufs, 0, sizeof(rqbufs));
    rqbufs.count = n;
    rqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    rqbufs.memory = V4L2_MEMORY_DMABUF;

    ret = ioctl(v4l2->fd, VIDIOC_REQBUFS, &rqbufs);
    if (ret < 0)
        ERROR("REQBUFS failed: %s\n", strerror(errno));

    DBG("allocated buffers = %d\n", rqbufs.count);

    return 0;

#else
    struct v4l2_requestbuffers reqbuf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_DMABUF,
            .count = n,
    };
    uint32_t i;
    int ret,dmafd;

    if (v4l2->v4l2bufs) {
        // maybe eventually need to support this?
        ERROR("already reqbuf'd");
        return -1;
    }

    ret = ioctl(v4l2->fd, VIDIOC_REQBUFS, &reqbuf);
    if (ret < 0) {
        ERROR("VIDIOC_REQBUFS failed: %s (%d)", strerror(errno), ret);
        return ret;
    }

    if ((reqbuf.count != n) ||
            (reqbuf.type != V4L2_BUF_TYPE_VIDEO_CAPTURE) ||
            (reqbuf.memory != V4L2_MEMORY_DMABUF)) {
        ERROR("unsupported..");
        return -1;
    }

#if 0
    v4l2->nbufs = reqbuf.count;
    v4l2->bufs = bufs;
    v4l2->v4l2bufs = calloc(v4l2->nbufs, sizeof(*v4l2->v4l2bufs));
    if (!v4l2->v4l2bufs) {
        ERROR("allocation failed");
        return -1;
    }

    for (i = 0; i < reqbuf.count; i++) {
        assert(bufs[i]->nbo == 1); /* TODO add multi-planar support */
        /* Call omap_bo_dmabuf only once, to export only once
         * Otherwise, each call will return duplicated fds
         * This way, every call to omap_bo_dmabuf will return a new fd
         * Which won't match with any previously exported fds
         * Instead, store dma fd in buf->fd[] */
        dmafd = omap_bo_dmabuf(bufs[i]->bo[0]);
        bufs[i]->fd[0] = dmafd;
        v4l2->v4l2bufs[i] = (struct v4l2_buffer){
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
                    .memory = V4L2_MEMORY_DMABUF,
                    .index = i,
                    .m.fd = dmafd,
        };
        MSG("Exported buffer fd = %d\n", dmafd);
        ret = ioctl(v4l2->fd, VIDIOC_QUERYBUF, &v4l2->v4l2bufs[i]);
        v4l2->v4l2bufs[i].m.fd = dmafd;
        if (ret) {
            ERROR("VIDIOC_QUERYBUF failed: %s (%d)", strerror(errno), ret);
            return ret;
        }
    }
#endif
#endif
    return 0;
}

int
v4l2_streamon(struct v4l2 *v4l2)
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int ret;

    ret = ioctl(v4l2->fd, VIDIOC_STREAMON, &type);

    if (ret) {
        ERROR("VIDIOC_STREAMON failed: %s (%d)", strerror(errno), ret);
    }

    return ret;
}

int
v4l2_streamoff(struct v4l2 *v4l2)
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int ret;

    ret = ioctl(v4l2->fd, VIDIOC_STREAMOFF, &type);

    if (ret) {
        ERROR("VIDIOC_STREAMOFF failed: %s (%d)", strerror(errno), ret);
    }

    return ret;
}

int
v4l2_qbuf(struct v4l2 *v4l2, int dmafd, int index)
{
#if 1
    int ret;
    struct v4l2_buffer buf;

    DBG("v4l2 buffer queue\n");

    memset(&buf, 0, sizeof buf);
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.index = index;
    buf.m.fd = dmafd; //vpe->input_buf_dmafd[index];
    //printf("%d, %d(%d), %d(%d), %d, %d\n", 
    //  vipfd, buf.type, V4L2_BUF_TYPE_VIDEO_CAPTURE, buf.memory, V4L2_MEMORY_DMABUF, buf.index, buf.m.fd);

    ret = ioctl(v4l2->fd, VIDIOC_QBUF, &buf);
    if (ret < 0)
        ERROR("QBUF failed: %s, index = %d\n", strerror(errno), index);

    return 0;
#else
    struct v4l2_buffer *v4l2buf = NULL;
    int i, ret, fd;

    struct timeval disp_time;
    struct timeval disp_lat;
    pthread_t tid;

    assert(buf->nbo == 1); /* TODO add multi-planar support */

    fd = buf->fd[0];

    for (i = 0; i < v4l2->nbufs; i++) {
        if (v4l2->v4l2bufs[i].m.fd == fd) {
            v4l2buf = &v4l2->v4l2bufs[i];
        }
    }

    tid = pthread_self();
    gettimeofday(&disp_time, NULL);
    timersub(&disp_time, &v4l2buf->timestamp, &disp_lat);
    if(v4l2buf->timestamp.tv_sec)
        DBG("%x: DISP latency = %d.%6d", (unsigned int)tid, (int)disp_lat.tv_sec, (int)disp_lat.tv_usec);

    if (!v4l2buf) {
        ERROR("invalid buffer");
        return -1;
    }

    ret = ioctl(v4l2->fd, VIDIOC_QBUF, v4l2buf);
    v4l2buf->m.fd = buf->fd[0];
    if (ret) {
        ERROR("VIDIOC_QBUF failed: %s (%d)", strerror(errno), ret);
    }
    return ret;
#endif

}

int
v4l2_dqbuf(struct v4l2 *v4l2, int* field)
{
    struct v4l2_buffer v4l2buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_DMABUF,
    };
    int ret;

    ret = ioctl(v4l2->fd, VIDIOC_DQBUF, &v4l2buf);
    if (ret) {
        ERROR("VIDIOC_DQBUF failed: %s (%d)", strerror(errno), ret);
    }

#if 1
    DBG("vip: DQBUF idx = %d, field = %s\n", v4l2buf.index,
        v4l2buf.field == V4L2_FIELD_TOP? "Top" : "Bottom");
    *field = v4l2buf.field;

    return v4l2buf.index;
#else
    buf = v4l2->bufs[v4l2buf.index];
    v4l2->v4l2bufs[v4l2buf.index].timestamp = v4l2buf.timestamp;

    assert(buf->nbo == 1); /* TODO add multi-planar support */

    tid = pthread_self();
    gettimeofday(&dq_time, NULL);
    timersub(&dq_time, &v4l2buf.timestamp, &dq_lat);
    DBG("%x: DQ latency = %d.%6d", (unsigned int)tid, (int)dq_lat.tv_sec, (int)dq_lat.tv_usec);

    return buf;
#endif
}
