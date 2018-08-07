
#ifndef DISPLAY_KMS_H_
#define DISPLAY_KMS_H_

#include <omap_drm.h>
#include <omap_drmif.h>
#define HAVE_CONFIG_H

#include "util.h"

struct plane {
    uint32_t id;
    uint32_t x;
    uint32_t y;

    uint32_t xres;
    uint32_t yres;
    uint32_t fb_id;
    uint32_t z_val;
    uint32_t glo_alp;
    uint32_t pre_mul_alp;
};

struct display {
    int fd;
    uint32_t width, height;
    struct omap_device *dev;
    struct list unlocked;
    struct rate_control rtctl;

    struct buffer ** (*get_buffers)(struct display *disp, uint32_t n);
    struct buffer ** (*get_vid_buffers)(struct display *disp,
            uint32_t n, uint32_t fourcc, uint32_t w, uint32_t h);
    int (*post_buffer)(struct display *disp, struct buffer *buf);
    int (*post_vid_buffer)(struct display *disp, struct buffer *buf,
            uint32_t x, uint32_t y, uint32_t w, uint32_t h);
    void (*close)(struct display *disp);
    void (*disp_free_buf) (struct display *disp, uint32_t n);

    bool multiplanar;   /* True when Y and U/V are in separate buffers. */
    struct buffer **buf;

    struct plane overlay_p;
    struct buffer *overlay_p_bo;
};

/* Print display related help */
void disp_usage(void);

/* Open display.. X11 or KMS depending on cmdline args, environment,
 * and build args
 */
struct display * disp_open(int argc, char **argv);

/* free allocated buffer */
void disp_free_buffers(struct display *disp, uint32_t n);
/* Close display */
static inline void
disp_close(struct display *disp)
{
    disp->close(disp);
}

/* Get normal RGB/UI buffers (ie. not scaled, not YUV) */
static inline struct buffer **
disp_get_buffers(struct display *disp, uint32_t n)
{
    return disp->get_buffers(disp, n);
}

/* Get video/overlay buffers (ie. can be YUV, scaled, etc) */
struct buffer ** disp_get_vid_buffers(struct display *disp, uint32_t n,
        uint32_t fourcc, uint32_t w, uint32_t h);

/* flip to / post the specified buffer */
int
disp_post_buffer(struct display *disp, struct buffer *buf);

/* flip to / post the specified video buffer */
int
disp_post_vid_buffer(struct display *disp, struct buffer *buf,
        uint32_t x, uint32_t y, uint32_t w, uint32_t h);

/* Get plane (id = 1) for every connector and update the overlay */
//int
//get_overlay_plane(struct display *disp, struct buffer *buf);

/* allocate a buffer from pool created by disp_get_vid_buffers() */
struct buffer * disp_get_vid_buffer(struct display *disp);
/* free to video buffer pool */
void disp_put_vid_buffer(struct display *disp, struct buffer *buf);

/* helper to setup the display for apps that just need video with
 * no flipchain on the GUI layer
 */
struct buffer * disp_get_fb(struct display *disp);

void fill(struct buffer *buf, int i);

struct buffer *alloc_buffer(struct display *disp, uint32_t fourcc, uint32_t w, uint32_t h, bool addfb);

void set_z_order(struct display *disp, uint32_t plane_id);
void set_global_alpha(struct display *disp, uint32_t plane_id);
void set_pre_multiplied_alpha(struct display *disp, uint32_t plane_id);
void alloc_overlay_plane(struct display *disp, uint32_t fourcc, int x, int y, int w, int h);
void free_overlay_plane(struct display *disp);
void update_overlay_disp(struct display *disp);
int get_framebuf(struct buffer *buf, unsigned char** ppfbuf);

#endif
