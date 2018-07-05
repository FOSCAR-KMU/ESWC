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

#ifndef UTIL_H_
#define UTIL_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <assert.h>

#include "list.h"

#define FOURCC(a, b, c, d) ((uint32_t)(uint8_t)(a) | ((uint32_t)(uint8_t)(b) << 8) | ((uint32_t)(uint8_t)(c) << 16) | ((uint32_t)(uint8_t)(d) << 24 ))
#define FOURCC_STR(str)    FOURCC(str[0], str[1], str[2], str[3])

/* Dynamic debug. */
#define DBG(fmt, ...) \
        do { if (debug) fprintf(stderr, fmt "\n", ##__VA_ARGS__); } while (0)

#define MSG(fmt, ...) \
        do { fprintf(stderr, fmt "\n", ##__VA_ARGS__); } while (0)
#define ERROR(fmt, ...) \
        do { fprintf(stderr, "ERROR:%s:%d: " fmt "\n", __func__, __LINE__, ##__VA_ARGS__); } while (0)

#ifndef container_of
#define container_of(ptr, type, member) \
    (type *)((char *)(ptr) - (char *) &((type *)0)->member)
#endif

#ifndef MIN
#  define MIN(a,b)     (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b)     (((a) > (b)) ? (a) : (b))
#endif

#ifndef PAGE_SHIFT
#  define PAGE_SHIFT 12
#endif

#ifndef PAGE_SIZE
#  define PAGE_SIZE (1 << PAGE_SHIFT)
#endif

/* align x to next highest multiple of 2^n */
#define ALIGN2(x,n)   (((x) + ((1 << (n)) - 1)) & ~((1 << (n)) - 1))

/* Display Interface:
 *
 * Could be either KMS or X11 depending on build and
 * environment.  Some of details of opening/connecting, allocating buffers,
 * etc, differ.  The intention is just to provide as simple as possible
 * abstraction to avoid lots of duplication in each test app to handle
 * both cases.
 */

struct buffer {
    uint32_t fourcc, width, height;
    int nbo;
    struct omap_bo *bo[4];
    uint32_t pitches[4];
    struct list unlocked;
    bool multiplanar;   /* True when Y and U/V are in separate buffers. */
    int fd[4];          /* dmabuf */
    bool noScale;
};

/* Other utilities..
 */
extern int debug;

/* State variables, used to maintain the playback rate. */
struct rate_control {
    int fps;        /* When > zero, we maintain playback rate. */
    long last_frame_mark;   /* The time when the last frame was displayed,
                 * as returned by the mark() function. */
    int usecs_to_sleep; /* Number of useconds we have slep last frame. */
};

int check_args(int argc, char **argv);
void maintain_playback_rate(struct rate_control *p);

#include <sys/time.h>
static inline long
mark(long *last)
{
    struct timeval t;
    gettimeofday(&t, NULL);
    if (last) {
        long delta = t.tv_usec - *last;

        /* Handle the case, where the seconds have changed.
         * TODO: keep the whole timeval struct, to be able to cope with
         * more than one second deltas? */
        if (t.tv_usec < *last)
            delta += 1000000;

        *last = t.tv_usec;
        return delta;
    }
    return t.tv_usec;
}
#endif /* UTIL_H_ */
