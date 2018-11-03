#ifndef DRAWING_H_
#define DRAWING_H_ 

#include <stdint.h>

typedef enum {
    FORMAT_BGR565,
    FORMAT_RGB565,
    FORMAT_BGR888,
    FORMAT_RGB888,
    FORMAT_ABGR8888,
    FORMAT_ARGB8888,
    FORMAT_BGRA8888,
    FORMAT_RGBA8888,
    // TODO .. add ohter foramt.. ex: yuyv, nv12, etc..
    
    FORMAT_MAX
} PixelFormat;

typedef struct _FrameBuffer
{
    PixelFormat format;
    uint32_t stride;

    unsigned char* buf;
} FrameBuffer;

PixelFormat draw_get_pixel_foramt(int fourcc);
void drawPixel(FrameBuffer* pFrame, uint32_t px, uint32_t py, uint32_t color);
void drawLine(FrameBuffer* pFrame, uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t color);
void drawChar(FrameBuffer* pFrame, char c, uint32_t startx, uint32_t starty, uint32_t size, uint32_t color);
void drawString(FrameBuffer* pFrame, char* str, uint32_t startx, uint32_t starty, uint32_t size, uint32_t color);
void drawRect(FrameBuffer* pFrame, uint32_t startx, uint32_t starty, uint32_t w, uint32_t h, uint32_t color);

#endif //DRAWING_H_

