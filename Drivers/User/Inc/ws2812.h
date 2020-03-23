#ifndef __WS2812_H
#define __WS2812_H

#include "main.h"
#define WS2812_DATA_BIT_0 35U //450ns
#define WS2812_DATA_BIT_1 65U //800ns
#define WS2812_PIXEL 8
typedef struct
{
	 uint16_t head[40];												//帧头，保持40*1.25us低电平	REST Signal(Keep Low Level 40*1.25us)
	 uint16_t data[24 * WS2812_PIXEL];				//数据，RGB*LED数量					RGB	Data Base,(R(8bit)+G(8bit)+B(8bit))* LED Numbers
	 uint16_t tail[1];					  						//帧尾，保持低电平					Must Keep Low Level
}ws2812_frame_struct_t;

typedef struct
{
	uint8_t G;
	uint8_t R;
	uint8_t B;
}ws2812_pixel_struct_t;

void ws2812_update(void);

#endif //__WS2812_H
