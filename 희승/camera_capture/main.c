#include <signal.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <termios.h>
#include <sys/time.h>
#include <errno.h>
#include <syslog.h>

#include "util.h"

#include "display-kms.h"
#include "v4l2.h"
#include "input_cmd.h"

#define NBUF 6 // v4l camera buf count

#define CAPTURE_IMG_W       1280 // camera resolution
#define CAPTURE_IMG_H       720  // camera resolution
#define CAPTURE_IMG_SIZE    (CAPTURE_IMG_W*CAPTURE_IMG_H*2) // YUYU : 16bpp
#define CAPTURE_IMG_FORMAT  "uyvy" // camera resolution

static uint32_t getfourccformat (char *format)
{
    if (strcmp (format, "rgb24") == 0) {
        return FOURCC('R','G','B','3');
    } else if (strcmp (format, "bgr24") == 0) {
        return FOURCC('B','G','R','3');
    } else if (strcmp (format, "argb32") == 0) {
        return FOURCC('A','R','2','4');
    } else if (strcmp (format, "abgr32") == 0) {
        return FOURCC('R','A','2','4');
    } else if (strcmp (format, "yuyv") == 0) {
        return FOURCC('Y','U','Y','V');
    } else if (strcmp (format, "uyvy") == 0) {
        return FOURCC('U','Y','V','Y');
    } else if (strcmp (format, "nv12") == 0) {
        return FOURCC('N','V','1','2');
    } else {
        MSG("not yet support format..");
    }

    return 0;
}


int main(int argc, char **argv)
{
    struct display *disp;
    struct v4l2 *v4l2;
    struct buffer **buffers, *capt;

    int disp_argc = 3;
    char* disp_argv[] = {"dummy", "-s", "4:480x272", "\0"};
    int i, ret = 0;

    uint32_t fourcc;
    printf("-- 1_camera_dump_disp example Start --\n");


    disp = disp_open(disp_argc, disp_argv);
    if (!disp) {
        ERROR("disp open error!");
        return 1;
    }

    fourcc = getfourccformat(CAPTURE_IMG_FORMAT);
    v4l2 = v4l2_open(fourcc, CAPTURE_IMG_W, CAPTURE_IMG_H);
    if (!v4l2) {
        ERROR("v4l2 open error!");
        disp_close(disp);
        return 1;
    }

    MSG ("Input(Camera) = %d x %d (%.4s)\nOutput(LCD) = %d x %d (%.4s)",
        CAPTURE_IMG_W, CAPTURE_IMG_H, (char*)&fourcc,
        CAPTURE_IMG_H, CAPTURE_IMG_H, (char*)&fourcc);

    // display buffer alloc
    buffers = disp_get_vid_buffers(disp, NBUF, getfourccformat(CAPTURE_IMG_FORMAT), CAPTURE_IMG_W, CAPTURE_IMG_H);

    ret = v4l2_reqbufs(v4l2, buffers, NBUF);

    for (i = 0; i < NBUF; i++) {
      v4l2_qbuf(v4l2, buffers[i]);
    }

    ret = v4l2_streamon(v4l2);

    //Normally, camera capture and display output
    while(true) {
      capt = v4l2_dqbuf(v4l2);

      capt->noScale = true;

      ret = disp_post_vid_buffer(disp, capt,
        0, 0, CAPTURE_IMG_W, CAPTURE_IMG_H);
      if (ret) {
        ERROR("Post buffer failed");
      }

      v4l2_qbuf(v4l2, capt);
    }
    v4l2_streamoff(v4l2);

    return ret;
}
