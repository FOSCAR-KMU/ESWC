
#include <stdio.h>
#include <stdbool.h>

#include "util.h"
#include "drawing.h"
#include "font_8x8.h"

#ifdef DEBUG_ENABLE
    #define DBGLOG(fmt,args...) fprintf(stderr, fmt "\n",##args)
#else
    #define DBGLOG(fmt,args...)
#endif

typedef struct _DrawColor
{
    uint8_t b;
    uint8_t g;
    uint8_t r;
    uint8_t a;
} DrawColor;

static bool get_char_pixel(char c, uint32_t x, uint32_t y)
{
    uint8_t bits = fontdata_8x8[8 * c + y];
    bool bit = (bits >> (7 - x)) & 1;

    return bit;
}

static uint32_t rgb888(DrawColor* pColor)
{
    return (pColor->r << 16) | (pColor->g << 8) | (pColor->b << 0);
}

static uint32_t bgr888(DrawColor* pColor)
{
    return (pColor->b << 16) | (pColor->g << 8) | (pColor->r << 0);
}

static uint32_t argb8888(DrawColor* pColor)
{
    return (pColor->a << 24) | (pColor->r << 16) | (pColor->g << 8) | (pColor->b << 0);
}

static uint32_t abgr8888(DrawColor* pColor)
{
    return (pColor->a << 24) | (pColor->b << 16) | (pColor->g << 8) | (pColor->r << 0);
}

static uint32_t rgba8888(DrawColor* pColor)
{
    return (pColor->r << 24) | (pColor->g << 16) | (pColor->b << 8) | (pColor->a << 24);
}

static uint32_t bgra8888(DrawColor* pColor)
{
    return (pColor->b << 24) | (pColor->g << 16) | (pColor->r << 8) | (pColor->a << 0);
}

static uint16_t rgb565(DrawColor* pColor)
{
    return ((pColor->r >> 3) << 11) | ((pColor->g >> 2) << 5) | ((pColor->b >> 3) << 0);
}

static uint16_t bgr565(DrawColor* pColor)
{
    return ((pColor->b >> 3) << 11) | ((pColor->g >> 2) << 5) | ((pColor->r >> 3) << 0);
}


/**
  * @brief  Get pixel format
  * @param  fourcc: color format of FOURCC type
  * @retval The enum value of the PixelFormat
  */
PixelFormat draw_get_pixel_foramt(int fourcc)
{
    PixelFormat format;

    switch(fourcc) {
        case FOURCC('R','G','1','6') :
            format = FORMAT_RGB565;
            break;
        case FOURCC('B','G','1','6') :
            format = FORMAT_BGR565;
            break;
        case FOURCC('R','G','2','4'):
            format = FORMAT_RGB888;
            break;
        case FOURCC('B','G','2','4'):
            format = FORMAT_BGR888;
            break;

        case FOURCC('A','R','2','4'):
            format = FORMAT_ARGB8888;
            break;

        case FOURCC('A','B','2','4'):
            format = FORMAT_ABGR8888;
            break;

        case FOURCC('R','A','2','4'):
            format = FORMAT_RGBA8888;
            break;

        case FOURCC('R','B','2','4'):
            format = FORMAT_BGRA8888;
            break;

        case FOURCC('U','Y','V','Y'):
        case FOURCC('Y','U','Y','V'):
        case FOURCC('N','V','1','2'):
        case FOURCC('I','4','2','0'):
            fprintf(stderr, "ERROR: %s not yet support!\n", __func__);
            format = FORMAT_MAX;
            break;

        default :
            fprintf(stderr, "ERROR: %s Invalid param\n", __func__);
            format = FORMAT_MAX;
            break;
    }

    DBGLOG("[%s] get pixel format:%d", __func__, format);

    return format;
}

/**
  * @brief  Draw a pixel to frame buffer
  * @param  pFrame: pointer to parameter of FrameBuffer structure
                 px : x position of the pixel
                 py : y position of the pixel
                 color : color format(PixelFormat) of the pixel. 
  * @retval The enum value of the PixelFormat
  */
void drawPixel(FrameBuffer* pFrame, uint32_t px, uint32_t py, uint32_t color)
{
    DrawColor RgbColor;

    RgbColor.a = (color >> 24 & 0xff);
    RgbColor.r = (color >> 16 & 0xff);
    RgbColor.g = (color >> 8 & 0xff);
    RgbColor.b = (color & 0xff);

    if(pFrame == NULL) {
        ERROR("Invalid param");
        return;
    }

    switch (pFrame->format) {
        case FORMAT_ARGB8888:
        {
            uint32_t *p = (uint32_t*)(pFrame->buf + pFrame->stride * py + px * 4);
            *p = color;//argb8888(&RgbColor);
            break;
        }
        case FORMAT_ABGR8888:
        {
            uint32_t *p = (uint32_t*)(pFrame->buf + pFrame->stride * py + px * 4);
            *p = abgr8888(&RgbColor);
            break;
        }
        case FORMAT_RGBA8888:
        {
            uint32_t *p = (uint32_t*)(pFrame->buf + pFrame->stride * py + px * 4);
            *p = rgba8888(&RgbColor);
            break;
        }
        case FORMAT_BGRA8888:
        {
            uint32_t *p = (uint32_t*)(pFrame->buf + pFrame->stride * py + px * 4);
            *p = bgra8888(&RgbColor);
            break;
        }

        case FORMAT_RGB888:
        {
            uint8_t *p = pFrame->buf + pFrame->stride * py + px * 3;
            p[0] = RgbColor.b;
            p[1] = RgbColor.g;
            p[2] = RgbColor.r;
            break;
        }
        case FORMAT_BGR888:
        {
            uint8_t *p = pFrame->buf + pFrame->stride * py + px * 3;
            p[0] = RgbColor.r;
            p[1] = RgbColor.g;
            p[2] = RgbColor.b;
            break;
        }
        case FORMAT_RGB565:
        {
            uint16_t *p = (uint16_t*)(pFrame->buf + pFrame->stride * py + px * 2);
            *p = rgb565(&RgbColor);
            break;
        }
        case FORMAT_BGR565:
        {
            uint16_t *p = (uint16_t*)(pFrame->buf + pFrame->stride * py + px * 2);
            *p = bgr565(&RgbColor);
            break;
        }

        default :
            ERROR("Invalid param");
            break;
    }
}

/**
  * @brief  Draw a rectangle to frame buffer
  * @param  pFrame: pointer to parameter of FrameBuffer structure
                 startx : start position of x axis for the rectangle
                 starty : start position of y axis for the rectangle
                 w : width of the rectangle
                 h : height of the rectangle
                 color : color format(PixelFormat) of the rectangle. 
  * @retval none
  */
void drawRect(FrameBuffer* pFrame, uint32_t startx, uint32_t starty, uint32_t w, uint32_t h, uint32_t color)
{
    unsigned i, j;

    if(pFrame == NULL) {
        ERROR("Invalid param");
        return;
    }

    DBGLOG("[%s] xpos:%d ypos:%d, stride:%d", __func__, startx, starty, pFrame->stride);
    DBGLOG("[%s] format:%d color:0x%08x", __func__, pFrame->format, color);

    switch (pFrame->format) {
        case FORMAT_BGR565 :
        case FORMAT_RGB565 :
        case FORMAT_BGR888 :
        case FORMAT_RGB888 :
        case FORMAT_ABGR8888 :
        case FORMAT_ARGB8888 :
        case FORMAT_BGRA8888 :
        case FORMAT_RGBA8888 :
            for (j = 0; j < h; j++) {
                for (i = 0; i < w; i++) {
                    drawPixel(pFrame, startx + i, starty + j, color);
                }
            }
            break;

        default :
            ERROR("Invalid param");
            break;
    }
}

/**
  * @brief  Draw a line to frame buffer
  * @param  pFrame: pointer to parameter of FrameBuffer structure
                 x1 : start position of x axis for the line
                 y1 : start position of y axis for the line
                 x2 : end position of x axis for the line
                 y2 : end position of y axis for the line
                 color : color format(PixelFormat) of the line. 
  * @retval none
  */
void drawLine(FrameBuffer* pFrame, uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t color)
{
    int i,dx,dy,sdx,sdy,dxabs,dyabs,x,y,px,py;

    if(pFrame == NULL) {
        ERROR("Invalid param");
        return;
    }

    switch (pFrame->format) {
        case FORMAT_BGR565 :
        case FORMAT_RGB565 :
        case FORMAT_BGR888 :
        case FORMAT_RGB888 :
        case FORMAT_ABGR8888 :
        case FORMAT_ARGB8888 :
        case FORMAT_BGRA8888 :
        case FORMAT_RGBA8888 :
            dx=x2-x1;           //Delta x
            dy=y2-y1;           //Delta y
            dxabs=abs(dx);      //Absolute delta
            dyabs=abs(dy);      //Absolute delta
            sdx=(dx>0)?1:-1; //signum function
            sdy=(dy>0)?1:-1; //signum function
            x=dyabs>>1;
            y=dxabs>>1;
            px=x1;
            py=y1;

            if (dxabs>=dyabs)
            {
                for(i=0;i<dxabs;i++)
                {
                    y+=dyabs;
                    if (y>=dxabs)
                    {
                        y-=dxabs;
                        py+=sdy;
                    }
                    px+=sdx;
                    drawPixel(pFrame, px, py, color);
                }
            }
            else
            {
                for(i=0;i<dyabs;i++)
                {
                    x+=dxabs;
                    if (x>=dyabs)
                    {
                        x-=dyabs;
                        px+=sdx;
                    }
                    py+=sdy;
                    drawPixel(pFrame, px, py, color);
                }
            }
            break;

        default :
            ERROR("Invalid param");
            break;
    }
}

/**
  * @brief  Draw a character to frame buffer
  * @param  pFrame: pointer to parameter of FrameBuffer structure
                 c :  character to draw
                 startx : start position of y axis for the character
                 starty : start position of x axis for the character
                 size : font size (This is not used. Fixed to 8x8)
                 color : color format(PixelFormat) of the character. 
  * @retval none
  */
void drawChar(FrameBuffer* pFrame, char c, uint32_t startx, uint32_t starty, uint32_t size, uint32_t color)
{
    unsigned x, y;

    if(pFrame == NULL) {
        ERROR("Invalid param");
        return;
    }

    switch(pFrame->format) {
        case FORMAT_BGR565 :
        case FORMAT_RGB565 :
        case FORMAT_BGR888 :
        case FORMAT_RGB888 :
        case FORMAT_ABGR8888 :
        case FORMAT_ARGB8888 :
        case FORMAT_BGRA8888 :
        case FORMAT_RGBA8888 :
            for (y = 0; y < 8; y++) {
                for (x = 0; x < 8; x++) {
                    bool b = get_char_pixel(c, x, y);
                    //if(b)
                        drawPixel(pFrame, startx + x, starty + y, b ? color : 0x00000000);
                }
            }
            break;

        default :
            ERROR("Invalid param");
            break;
    }
}

/**
  * @brief  Draw a string to frame buffer
  * @param  pFrame: pointer to parameter of FrameBuffer structure
                 str : pointer to parameter of string
                 startx : start position of y axis for the string
                 starty : start position of x axis for the string
                 size : font size (This is not used. Fixed to 8x8)
                 color : color format(PixelFormat) of the character. 
  * @retval none
  */
void drawString(FrameBuffer* pFrame, char* str, uint32_t startx, uint32_t starty, uint32_t size, uint32_t color)
{
    uint32_t i;
    uint32_t len;
    
    len = strlen(str);

    DBGLOG("[%s] startx:%d starty:%d, stride:%d, str_len : %d", __func__, startx, starty, pFrame->stride, len);
    DBGLOG("[%s] format:%d color:0x%08x", __func__, pFrame->format, color);

    if(pFrame == NULL) {
        ERROR("Invalid param");
        return;
    }

    for(i = 0; i < len; i++)
        drawChar(pFrame, str[i], (startx + 8 * i), starty, size, color);
}

