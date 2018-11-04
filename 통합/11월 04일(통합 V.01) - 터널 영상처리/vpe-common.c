/*
 *  Copyright (c) 2013-2014, Texas Instruments Incorporated
 *  Author: alaganraj <alaganraj.s@ti.com>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  *  Neither the name of Texas Instruments Incorporated nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 *  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 *  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 *  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 *  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  Contact information for paper mail:
 *  Texas Instruments
 *  Post Office Box 655303
 *  Dallas, Texas 75265
 *  Contact information:
 *  http://www-k.ext.ti.com/sc/technical-support/product-information-centers.htm?
 *  DCMP=TIHomeTracking&HQS=Other+OT+home_d_contact
 *  ============================================================================
 *
 */

/*
 * @File        vpe-common.c
 * @Brief       vpe specific common functions, used to integrate vpe 
 *      with other modules.
 *
 *      Input buffer must be allocated in application, queue it to vpe 
 *      by passing buffer index
 *      
 *      Output buffer allocated in vpe_output_init() as vpe output intended
 *      to display on LCD.
 */
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>

#include <linux/videodev2.h>
#include <linux/v4l2-controls.h>

#include <sys/mman.h>
#include <sys/ioctl.h>

#include <xf86drm.h>
#include <omap_drm.h>
#include <omap_drmif.h>

#include "util.h"
#include "vpe-common.h"


#define pexit(fmt, arg...) { \
        printf(fmt, ## arg); \
        exit(1); \
}

#define V4L2_CID_TRANS_NUM_BUFS         (V4L2_CID_PRIVATE_BASE)


//#define vpe_debug

#ifdef vpe_debug
#define dprintf(fmt, arg...) printf(fmt, ## arg)
#else
#define dprintf(fmt, arg...) do {} while(0)
#endif

/**
 *****************************************************************************
 * @brief:  open the device
 *
 * @return: vpe  struct vpe pointer 
 *****************************************************************************
*/
struct vpe *vpe_open(void)
{
    char devname[20] = "/dev/video0";
    struct vpe *vpe;

    vpe = calloc(1, sizeof(*vpe));

    vpe->fd =  open(devname, O_RDWR);
    if(vpe->fd < 0) {
            pexit("Cant open %s\n", devname);
            free(vpe);
            vpe = NULL;
    }
    return vpe;
}

/**
 *****************************************************************************
 * @brief:  close the device and free memory
 *
 * @param:  vpe  struct vpe pointer
 *
 * @return: 0 on success 
 *****************************************************************************
*/
int vpe_close(struct vpe *vpe)
{
    close(vpe->fd);
    free(vpe);
    
    return 0;
}

/**
 *****************************************************************************
 * @brief:  fills 4cc, size, coplanar, colorspace based on command line input 
 *
 * @param:  format  char pointer
 * @param:  image  struct image_params pointer
 *
 * @return: 0 on success 
 *****************************************************************************
*/
int describeFormat (char *format, struct image_params *image)
{
        image->size   = -1;
        image->fourcc = -1;
        if (strcmp (format, "rgb24") == 0) {
                image->fourcc = V4L2_PIX_FMT_RGB24;
                image->size = image->height * image->width * 3;
                image->coplanar = 0;
                image->colorspace = V4L2_COLORSPACE_SRGB;

        } else if (strcmp (format, "bgr24") == 0) {
                image->fourcc = V4L2_PIX_FMT_BGR24;
                image->size = image->height * image->width * 3;
                image->coplanar = 0;
                image->colorspace = V4L2_COLORSPACE_SRGB;

        } else if (strcmp (format, "argb32") == 0) {
                image->fourcc = V4L2_PIX_FMT_RGB32;
                image->size = image->height * image->width * 4;
                image->coplanar = 0;
                image->colorspace = V4L2_COLORSPACE_SRGB;

        } else if (strcmp (format, "abgr32") == 0) {
                image->fourcc = V4L2_PIX_FMT_BGR32;
                image->size = image->height * image->width * 4;
                image->coplanar = 0;
                image->colorspace = V4L2_COLORSPACE_SRGB;

        } else if (strcmp (format, "yuv444") == 0) {
                image->fourcc = V4L2_PIX_FMT_YUV444;
                image->size = image->height * image->width * 3;
                image->coplanar = 0;
                image->colorspace = V4L2_COLORSPACE_SMPTE170M;

        } else if (strcmp (format, "yvyu") == 0) {
                image->fourcc = V4L2_PIX_FMT_YVYU;
                image->size = image->height * image->width * 2;
                image->coplanar = 0;
                image->colorspace = V4L2_COLORSPACE_SMPTE170M;

        } else if (strcmp (format, "yuyv") == 0) {
                image->fourcc = V4L2_PIX_FMT_YUYV;
                image->size = image->height * image->width * 2;
                image->coplanar = 0;
                image->colorspace = V4L2_COLORSPACE_SMPTE170M;

        } else if (strcmp (format, "uyvy") == 0) {
                image->fourcc = V4L2_PIX_FMT_UYVY;
                image->size = image->height * image->width * 2;
                image->coplanar = 0;
                image->colorspace = V4L2_COLORSPACE_SMPTE170M;

        } else if (strcmp (format, "vyuy") == 0) {
                image->fourcc = V4L2_PIX_FMT_VYUY;
                image->size = image->height * image->width * 2;
                image->coplanar = 0;
                image->colorspace = V4L2_COLORSPACE_SMPTE170M;

        } else if (strcmp (format, "nv16") == 0) {
                image->fourcc = V4L2_PIX_FMT_NV16;
                image->size = image->height * image->width * 2;
                image->coplanar = 0;
                image->colorspace = V4L2_COLORSPACE_SMPTE170M;

        } else if (strcmp (format, "nv61") == 0) {
                image->fourcc = V4L2_PIX_FMT_NV61;
                image->size = image->height * image->width * 2;
                image->coplanar = 0;
                image->colorspace = V4L2_COLORSPACE_SMPTE170M;

        } else if (strcmp (format, "nv12") == 0) {
                image->fourcc = V4L2_PIX_FMT_NV12;
                image->size = image->height * image->width * 1.5;
                image->coplanar = 1;
                image->colorspace = V4L2_COLORSPACE_SMPTE170M;

        } else if (strcmp (format, "nv21") == 0) {
                image->fourcc = V4L2_PIX_FMT_NV21;
                image->size = image->height * image->width * 1.5;
                image->coplanar = 1;
                image->colorspace = V4L2_COLORSPACE_SMPTE170M;

        } else {
                return 0;

        }

        return 1;
}

/**
 *****************************************************************************
 * @brief:  sets crop parameters 
 *
 * @param:  vpe  struct vpe pointer
 *
 * @return: 0 on success 
 *****************************************************************************
*/
static int set_crop(struct vpe *vpe)
{
    int ret = 0;
    
    if ((vpe->crop.c.top == 0) && (vpe->crop.c.left == 0) &&
        (vpe->crop.c.width == 0) && (vpe->crop.c.height == 0)) {
        dprintf("setting default crop params\n");
        vpe->crop.c.top = 0;
        vpe->crop.c.left = 0;
        vpe->crop.c.width = vpe->src.width;
        vpe->crop.c.height = vpe->src.height;
        vpe->crop.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    }

    ret = ioctl(vpe->fd, VIDIOC_S_CROP, &vpe->crop);
    if (ret < 0)
        pexit("error setting crop\n");
    
    return 0;
}

/**
 *****************************************************************************
 * @brief:  sets control, howmany jobs to be handled on multi instance 
 *
 * @param:  vpe  struct vpe pointer
 *
 * @return: 0 on success 
 *****************************************************************************
*/
static int set_ctrl(struct vpe *vpe)
{
    int ret;
    struct  v4l2_control ctrl;

    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.id = V4L2_CID_TRANS_NUM_BUFS;
    ctrl.value = vpe->translen;
    ret = ioctl(vpe->fd, VIDIOC_S_CTRL, &ctrl);
    if (ret < 0)
        pexit("vpe: S_CTRL failed\n");
    
    return 0;
}

/**
 *****************************************************************************
 * @brief:  Intialize the vpe input by calling set_control, set_format,
 *      set_crop, refbuf ioctls
 *
 * @param:  vpe  struct vpe pointer
 *
 * @return: 0 on success 
 *****************************************************************************
*/
int vpe_input_init(struct vpe *vpe)
{
    int ret;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers rqbufs;

    set_ctrl(vpe);
        
    memset(&fmt, 0, sizeof fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;

    ret = ioctl(vpe->fd, VIDIOC_G_FMT, &fmt);
    if (ret < 0)
        pexit( "vpe i/p: G_FMT_1 failed: %s\n", strerror(errno));

    fmt.fmt.pix_mp.width = vpe->src.width;
    fmt.fmt.pix_mp.height = vpe->src.height;
    fmt.fmt.pix_mp.pixelformat = vpe->src.fourcc;
    fmt.fmt.pix_mp.colorspace = vpe->src.colorspace;

    switch (vpe->deint) {
    case 1:
        fmt.fmt.pix_mp.field = V4L2_FIELD_ALTERNATE;
        break;
    case 2:
        fmt.fmt.pix_mp.field = V4L2_FIELD_SEQ_TB;
        break;
    case 0:
        fmt.fmt.pix_mp.field = V4L2_FIELD_TOP;
        break;
    default:
        fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
        break;
    }

    ret = ioctl(vpe->fd, VIDIOC_S_FMT, &fmt);
    if (ret < 0) {
        pexit( "vpe i/p: S_FMT failed: %s\n", strerror(errno));
    } else {
                vpe->src.size = fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
                vpe->src.size_uv = fmt.fmt.pix_mp.plane_fmt[1].sizeimage;
        }

    ret = ioctl(vpe->fd, VIDIOC_G_FMT, &fmt);
    if (ret < 0)
        pexit( "vpe i/p: G_FMT_2 failed: %s\n", strerror(errno));

    MSG("vpe i/p: G_FMT: width = %u, height = %u, 4cc = %.4s",
            fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
            (char*)&fmt.fmt.pix_mp.pixelformat);

    set_crop(vpe);

    memset(&rqbufs, 0, sizeof(rqbufs));
    rqbufs.count = NUMBUF;
    rqbufs.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    rqbufs.memory = V4L2_MEMORY_DMABUF;

    ret = ioctl(vpe->fd, VIDIOC_REQBUFS, &rqbufs);
    if (ret < 0)
        pexit( "vpe i/p: REQBUFS failed: %s\n", strerror(errno));

    vpe->src.numbuf = rqbufs.count;
    dprintf("vpe i/p: allocated buffers = %d\n", rqbufs.count);
    
    return 0;

}

/**
 *****************************************************************************
 * @brief:  Initialize vpe output by calling set_format, reqbuf ioctls.
 *      Also allocates buffer to display the vpe output. 
 *
 * @param:  vpe  struct vpe pointer
 *
 * @return: 0 on success 
 *****************************************************************************
*/
int vpe_output_init(struct vpe *vpe)
{
    int ret, i;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers rqbufs;

    memset(&fmt, 0, sizeof fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    ret = ioctl(vpe->fd, VIDIOC_G_FMT, &fmt);
    if (ret < 0)
        pexit( "vpe o/p: G_FMT_1 failed: %s\n", strerror(errno));

    fmt.fmt.pix_mp.width = vpe->dst.width;
    fmt.fmt.pix_mp.height = vpe->dst.height;
    fmt.fmt.pix_mp.pixelformat = vpe->dst.fourcc;
    fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    fmt.fmt.pix_mp.colorspace = vpe->dst.colorspace;

    ret = ioctl(vpe->fd, VIDIOC_S_FMT, &fmt);
    if (ret < 0)
        pexit( "vpe o/p: S_FMT failed: %s\n", strerror(errno));

    ret = ioctl(vpe->fd, VIDIOC_G_FMT, &fmt);
    if (ret < 0)
        pexit( "vpe o/p: G_FMT_2 failed: %s\n", strerror(errno));

    MSG("vpe o/p: G_FMT: width = %u, height = %u, 4cc = %.4s",
            fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height,
            (char*)&fmt.fmt.pix_mp.pixelformat);

    memset(&rqbufs, 0, sizeof(rqbufs));
    rqbufs.count = NUMBUF;
    rqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    rqbufs.memory = V4L2_MEMORY_DMABUF;

    ret = ioctl(vpe->fd, VIDIOC_REQBUFS, &rqbufs);
    if (ret < 0)
        pexit( "vpe o/p: REQBUFS failed: %s\n", strerror(errno));

    vpe->dst.numbuf = rqbufs.count;
    dprintf("vpe o/p: allocated buffers = %d\n", rqbufs.count);
    
    vpe->disp_bufs = disp_get_vid_buffers(vpe->disp, NUMBUF, vpe->dst.fourcc, 
                          vpe->dst.width, vpe->dst.height);
    if (!vpe->disp_bufs)
        pexit("allocating display buffer failed\n");

    for (i = 0; i < NUMBUF; i++) {
        vpe->output_buf_dmafd[i] = omap_bo_dmabuf(vpe->disp_bufs[i]->bo[0]);
        vpe->disp_bufs[i]->fd[0] = vpe->output_buf_dmafd[i];

        if(vpe->dst.coplanar) {
            vpe->output_buf_dmafd_uv[i] = omap_bo_dmabuf(vpe->disp_bufs[i]->bo[1]);
            vpe->disp_bufs[i]->fd[1] = vpe->output_buf_dmafd_uv[i];
        }
        /* display only image widthxheight, no full screen */
//      vpe->disp_bufs[i]->noScale = true;
        dprintf("vpe->disp_bufs_fd[%d] = %d\n", i, vpe->output_buf_dmafd[i]);
    }

    dprintf("allocating display buffer success\n");
    return 0;
}

/**
 *****************************************************************************
 * @brief:  queue buffer to vpe input 
 *
 * @param:  vpe  struct vpe pointer
 * @param:  index  buffer index to queue
 *
 * @return: 0 on success 
 *****************************************************************************
*/
int vpe_input_qbuf(struct vpe *vpe, int index)
{
    int ret;
    struct v4l2_buffer buf;
    struct v4l2_plane planes[2];

    dprintf("vpe: src QBUF (%d):%s field", vpe->field,
        vpe->field==V4L2_FIELD_TOP?"top":"bottom");

    memset(&buf, 0, sizeof buf);
    memset(&planes, 0, sizeof planes);

    planes[0].length = planes[0].bytesused = vpe->src.size;
    if(vpe->src.coplanar)
        planes[1].length = planes[1].bytesused = vpe->src.size_uv;

    planes[0].data_offset = planes[1].data_offset = 0;

    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.index = index;
    buf.m.planes = planes;
    buf.field = vpe->field;
    if(vpe->src.coplanar)
        buf.length = 2;
    else
        buf.length = 1;

    buf.m.planes[0].m.fd = vpe->input_buf_dmafd[index];
    if(vpe->src.coplanar)
        buf.m.planes[1].m.fd = vpe->input_buf_dmafd_uv[index];

    ret = ioctl(vpe->fd, VIDIOC_QBUF, &buf);
    if (ret < 0)
        pexit( "vpe i/p: QBUF failed: %s, index = %d\n",
            strerror(errno), index);

    return 0;
}

/**
 *****************************************************************************
 * @brief:  queue buffer to vpe output 
 *
 * @param:  vpe  struct vpe pointer
 * @param:  index  buffer index to queue
 *
 * @return: 0 on success 
 *****************************************************************************
*/
int vpe_output_qbuf(struct vpe *vpe, int index)
{
    int ret;
    struct v4l2_buffer buf;
    struct v4l2_plane planes[2];

    dprintf("vpe output buffer queue\n");

    memset(&buf, 0, sizeof buf);
    memset(&planes, 0, sizeof planes);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_DMABUF;
    buf.index = index;
    buf.m.planes = planes;
    if(vpe->dst.coplanar)
        buf.length = 2;
    else
        buf.length = 1;

    buf.m.planes[0].m.fd = vpe->output_buf_dmafd[index];

    if(vpe->dst.coplanar)
        buf.m.planes[1].m.fd = vpe->output_buf_dmafd_uv[index];

    ret = ioctl(vpe->fd, VIDIOC_QBUF, &buf);
    if (ret < 0)
        pexit( "vpe o/p: QBUF failed: %s, index = %d\n",
            strerror(errno), index);

    return 0;
}

/**
 *****************************************************************************
 * @brief:  start stream 
 *
 * @param:  fd  device fd
 * @param:  type  buffer type (CAPTURE or OUTPUT)
 *
 * @return: 0 on success 
 *****************************************************************************
*/
int vpe_stream_on(int fd, int type)
{
    int ret;

    ret = ioctl(fd, VIDIOC_STREAMON, &type);
    if (ret < 0)
        pexit("STREAMON failed,  %d: %s\n", type, strerror(errno));

    dprintf("stream ON: done! fd = %d,  type = %d\n", fd, type);

    return 0;
}

/**
 *****************************************************************************
 * @brief:  stop stream 
 *
 * @param:  fd  device fd
 * @param:  type  buffer type (CAPTURE or OUTPUT)
 *
 * @return: 0 on success 
 *****************************************************************************
*/
int vpe_stream_off(int fd, int type)
{
    int ret;

    ret = ioctl(fd, VIDIOC_STREAMOFF, &type);
    if (ret < 0)
        pexit("STREAMOFF failed, %d: %s\n", type, strerror(errno));

    dprintf("stream OFF: done! fd = %d,  type = %d\n", fd, type);

    return 0;
}

/**
 *****************************************************************************
 * @brief:  dequeue vpe input buffer  
 *
 * @param:  vpe  struct vpe pointer
 *
 * @return: buf.index index of dequeued buffer
 *****************************************************************************
*/
int vpe_input_dqbuf(struct vpe *vpe)
{
    int ret;
    struct v4l2_buffer buf;
    struct v4l2_plane planes[2];
    
    dprintf("vpe input dequeue buffer\n");
    
    memset(&buf, 0, sizeof buf);
    memset(&planes, 0, sizeof planes);

    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.memory = V4L2_MEMORY_DMABUF;
        buf.m.planes = planes;
    if(vpe->src.coplanar)
        buf.length = 2;
    else
        buf.length = 1;
    ret = ioctl(vpe->fd, VIDIOC_DQBUF, &buf);
    if (ret < 0)
        pexit("vpe i/p: DQBUF failed: %s\n", strerror(errno));

    dprintf("vpe i/p: DQBUF index = %d\n", buf.index);

    return buf.index;
}

/**
 *****************************************************************************
 * @brief:  dequeue vpe output buffer
 *
 * @param:  vpe  struct vpe pointer
 *
 * @return: buf.index index of dequeued buffer
 *****************************************************************************
*/
int vpe_output_dqbuf(struct vpe* vpe)
{
    int ret;
    struct v4l2_buffer buf;
    struct v4l2_plane planes[2];

    dprintf("vpe output dequeue buffer\n");

    memset(&buf, 0, sizeof buf);
    memset(&planes, 0, sizeof planes);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_DMABUF;
        buf.m.planes = planes;
    if(vpe->dst.coplanar)
        buf.length = 2;
    else
        buf.length = 1;
    ret = ioctl(vpe->fd, VIDIOC_DQBUF, &buf);
    if (ret < 0)
        pexit("vpe o/p: DQBUF failed: %s\n", strerror(errno));

    dprintf("vpe o/p: DQBUF index = %d\n", buf.index);

    return buf.index;
}

/**
  * @brief  Set vpe display buffer scale value. if noScale is false, it's full screen.
  * @param  vpe: pointer to parameter of struct vpe display
                 bfull: true/false about full screen 
  * @retval  none
  */
void vpe_output_fullscreen(struct vpe* vpe, bool bfull)
{
    int i;
    for(i=0; i<NUMBUF; i++)
        vpe->disp_bufs[i]->noScale = (bfull) ? false : true;
}


