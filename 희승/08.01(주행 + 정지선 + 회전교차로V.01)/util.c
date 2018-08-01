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

#include "util.h"

/* Dynamic debug. */
int debug = 0;

/* Maintain playback rate if fps > 0. */
void maintain_playback_rate(struct rate_control *p)
{
    long usecs_since_last_frame;
    int usecs_between_frames, usecs_to_sleep;

    if (p->fps <= 0)
        return;

    usecs_between_frames = 1000000 / p->fps;
    usecs_since_last_frame = mark(&p->last_frame_mark);
    DBG("fps: %.02f", 1000000.0 / usecs_since_last_frame);
    usecs_to_sleep = usecs_between_frames - usecs_since_last_frame + p->usecs_to_sleep;

    if (usecs_to_sleep < 0)
        usecs_to_sleep = 0;

    /* mark() has a limitation that >1s time deltas will make the whole
     * loop diverge. Workaround that limitation by clamping our desired sleep time
     * to a maximum. TODO: Remove when mark() is in better shape. */
    if (usecs_to_sleep >= 1000000)
        usecs_to_sleep = 999999;

    /* We filter a bit our rate adaptation, to avoid being too "choppy".
     * Adjust the "alpha" value as needed. */
    p->usecs_to_sleep = ((67 * p->usecs_to_sleep) + (33 * usecs_to_sleep)) / 100;

    if (p->usecs_to_sleep >= 1) {
        DBG("sleeping %dus", p->usecs_to_sleep);
        usleep(p->usecs_to_sleep);
    }
}

int
check_args(int argc, char **argv)
{
    int i;
    for (i = 1; i < argc; i++) {
        if (argv[i]) {
            ERROR("invalid arg: %s", argv[i]);
            return -1;
        }
    }
    return 0;
}

