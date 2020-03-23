#include "tim.h"
#include "ws2812.h"

ws2812_frame_struct_t ws2812_frame_buffer = {
	.head = {0},
	.data = {0},
	.tail = {0},
};

ws2812_pixel_struct_t pixel_arr[WS2812_PIXEL] = {
#if (WS2812_PIXEL > 7)
{.G = 255,.R = 255,.B = 255},	//pixel 8
#endif
#if (WS2812_PIXEL > 6)
{.G = 100,.R = 100,.B = 100},	//pixel 7
#endif
#if (WS2812_PIXEL > 5)
{.G = 0,.R = 255,.B = 255},		//pixel 6
#endif
#if (WS2812_PIXEL > 4)
{.G = 255,.R = 0,.B = 255},		//pixel 5
#endif
#if (WS2812_PIXEL > 3)
{.G = 255,.R = 255,.B = 0},		//pixel 4
#endif
#if (WS2812_PIXEL > 2)
{.G = 0,.R = 0,.B = 255},			//pixel 3
#endif
#if (WS2812_PIXEL > 1)
{.G = 0,.R = 255,.B = 0},			//pixel 2
#endif
#if (WS2812_PIXEL > 0)
{.G = 255,.R = 0,.B = 0},			//pixel 1
#endif
};

/**************************************************************
* ��������������ת����PWMռ�ձȶ�Ӧ��TIMx CCRx�Ĵ�����ֵ��
* ÿ���������ɫ�Ų�˳��Ϊ��BRG
* ע��BRG��ÿһ����ɫ��8bit��3����ɫ��24��bit��
**************************************************************/
void ws2812_data_TransTo_frame(ws2812_pixel_struct_t const *pixel_arr, ws2812_frame_struct_t* frame, uint32_t pixel_num)
{
	uint32_t i,j,k;
	
	if(pixel_num <= WS2812_PIXEL && pixel_arr != NULL && frame != NULL)	//��鴫��	check parameter
		{/*nothing to do*/}
	else
		{return;}
	
	for(i = 0; i < pixel_num; i++)
	{
		k = 24*i;
		for(j = 0; j < 8; j++)
		{
			frame->data[k+j] = \
			((pixel_arr[i].B & (0x80U>>j))?WS2812_DATA_BIT_1:WS2812_DATA_BIT_0);
			
			frame->data[k+j+8] = \
			((pixel_arr[i].R & (0x80U>>j))?WS2812_DATA_BIT_1:WS2812_DATA_BIT_0);
			
			frame->data[k+j+16] = \
			((pixel_arr[i].G & (0x80U>>j))?WS2812_DATA_BIT_1:WS2812_DATA_BIT_0);
		}
	}
}

/**************************************************************
* ����ת������������ͨ��DMAת����TIMx CCRx�Ĵ���������PWM
**************************************************************/
void ws2812_update(void)
{
	ws2812_data_TransTo_frame(pixel_arr, &ws2812_frame_buffer, WS2812_PIXEL);
	if(HAL_TIM_PWM_Start_DMA(&htim3,TIM_CHANNEL_4,(uint32_t *)&ws2812_frame_buffer,41+24*WS2812_PIXEL) != HAL_OK)
	{
		Error_Handler();
	}
}
