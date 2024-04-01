// https://blog.csdn.net/u014545515/article/details/38363511/
// https://www.renrendoc.com/paper/161650011.html


#include "HSP_TSL1401.h"
#include "Lab3.h"
#include "HSP_CAT9555.h"
#include<stdbool.h>

#define CCD_PIXELS 128

//led state
int led_state = 0;

ccd_t ccd_data_raw, ccd_data_old;
uint8_t CCD2PC[260];	// data to be sent to PC using seekfree protocol
uint16_t max_v, min_v;				// max/min value of the linear array
uint8_t max_v_index, min_v_index;	// array index of the max/min_v
int16_t max_dv, min_dv;				// max/min value of the linear array delta
uint8_t max_dv_index, min_dv_index;	// array index of the max/min_dv
int16_t delta_v[124];				// delta v of the linear array
extern volatile uint32_t sys_tick_counter;

void line_position_alert(int line_position);

void Lab3_test(void)
{
  	uint8_t index;
	uint16_t pw = 1500, pwt = 0;
	
  	// initialize LCD
	hsp_spi_init();
	hsp_tft18_init();
	hsp_tft18_clear(BLACK);
	
	// initialize ADC/CCD
	hsp_ccd_init();
	hsp_demo_frame_ccd();
	
	// initialize PWM channels for motor and r/c servos
	hsp_pwm_init();

	while(1)
	{
		hsp_ccd_snapshot(ccd_data_raw);
		
		min_v = 4095U;
		max_v = 0U;
		for(index=2; index<126; index++)
		{
			// calculate delta v
			delta_v[index-2] = ccd_data_raw[index+2] - ccd_data_raw[index]; // 2 rather than 1

			// find the max of the linear array
			if(ccd_data_raw[index] > max_v)
			{
				max_v = ccd_data_raw[index];
				max_v_index = index;
			}
			// find the min of the linear arry
			if(ccd_data_raw[index] < min_v)
			{
				min_v = ccd_data_raw[index];
				min_v_index = index;
			}
		}

		min_dv = 4095;
		max_dv = -4095;
		for(index=0; index<124; index++)
		{
			// find the max in the linear array
			if(delta_v[index] > max_dv)
			{
				max_dv = delta_v[index];
				max_dv_index = index;
			}
			// find the min in the linear arry
			if(delta_v[index] < min_dv)
			{
				min_dv = delta_v[index];
				min_dv_index = index;
			}
		}

		hsp_tft18_show_int8(0, 2, min_dv_index);
		hsp_tft18_show_int8(0, 3, max_dv_index);

		int actual_max_dv_index = max_dv_index + 2; 
		int actual_min_dv_index = min_dv_index + 2;
		//检测黑线位置
		int line_position = (actual_max_dv_index + actual_min_dv_index) / 2;
		hsp_ccd_show(ccd_data_raw, line_position);
		line_position_alert(line_position);
		// line_position_alert(10);

		// calculate the steering angle command, pulse width in unit of us
		pw = 1500  - (min_dv_index + max_dv_index - 124) * 5; //*5是为了增大灵敏度
		
		// apply steering angle limits
		if(2050 < pw)
			pw = 2050;
		if(1100 > pw)
			pw = 1100;
		// update steering angle command only on change
		if(!SW1())
		{
			if(pwt != pw)
			{
				hsp_servo_angle(SERVO1, pw);
				hsp_servo_angle(SERVO2, pw);
				hsp_servo_angle(SERVO3, pw);
				hsp_servo_angle(SERVO4, pw);
				pwt = pw;
			}
		}
		else
		{
			hsp_servo_angle(SERVO1, 1500);
			hsp_servo_angle(SERVO2, 1500);
			hsp_servo_angle(SERVO3, 1500);
			hsp_servo_angle(SERVO4, 1500);
		}
		hsp_tft18_show_int16(80, 3, pw);
	}
	
}

void Lab3_seekfree(void)
{
  	uint8_t index;
	uint16_t index2;
	
  	// initialize LCD
	hsp_spi_init();
	hsp_tft18_init();
	hsp_tft18_clear(BLACK);
	
	// initialize UART2 on OpenSDA
	hsp_uart_init();
	hsp_usart2_dma_config();
	
	// initialize ADC/CCD
	hsp_ccd_init();
	hsp_demo_frame_ccd();
	
	for(index=0; index<128; index++)
		ccd_data_raw[index] = (index<<5);

	// Header of CCD data frame, seekfree protocol
    CCD2PC[0] = 0x00;
	CCD2PC[1] = 0xFF;
	CCD2PC[2] = 0x01;
	CCD2PC[3] = 0x00;

	while(1)
	{
		hsp_ccd_snapshot(ccd_data_raw);
		// hsp_ccd_show(ccd_data_raw);

		if (!PUSH())		// PUSHed?
		{
			//send 128 points CCD data (128*2byte) to UART0, using seekfree protocol
			for(index2=0; index2<128; index2++)
			{
				CCD2PC[index2*2+4] = ccd_data_raw[index2] >> 8;	// Upper byte
				CCD2PC[index2*2+5] = ccd_data_raw[index2] & 0XFF;	// Lower byte
			}
			hsp_uart2_dma_send_ascii(CCD2PC, 260);
		}
	}
}

static int prev_line_position = -1;
// draw LinearCCD waveform by pixels: [32,128] ~ [160,64]
void hsp_ccd_show(ccd_t data,int line_position)
{
    uint8_t i=0;
	int led_position = line_position * 16 / CCDPIXEL; // 将黑线位置映射到16位LED光柱上

    for(i=0; i<CCDPIXEL; i++)
	{
        hsp_tft18_draw_pixel(32+i, 128-(ccd_data_old[i]>>6), GRAY1);
        hsp_tft18_draw_pixel(32+i, 128-(data[i]>>6), BLUE);			// data/64 : 0~4095 -> 0~64
        ccd_data_old[i] = data[i];
	}

	// 如果黑线位置有效，则在LCD上显示red line，并更新LED光柱状态
	if (line_position >= 0) {
		// clear previous line
		hsp_tft18_draw_line_v(32+prev_line_position, 128-64, 64, GRAY1);
		// 绘制垂直线段
		hsp_tft18_draw_line_v(32 + line_position, 128 - 64, 64, RED);

		prev_line_position = line_position;
		// 映射黑线位置到16位LED光柱的范围
		int led_position = line_position * 16 / CCD_PIXELS; // 将128点映射到16位LED
		uint16_t led_to_light = (1 << led_position); 
		hsp_cat9555_ledbar(led_to_light);
	}
	else {
		// 如果没有检测到黑线，可以选择熄灭所有LED，或者采取其他适当的提示方式
		hsp_cat9555_ledbar(0); // 熄灭所有LED
	}



}


void hsp_demo_frame_ccd(void)
{
    hsp_tft18_show_str_color(1, 0, "ExpT:       Max:    ", WHITE, BLACK);
    hsp_tft18_show_str_color(1, 1, "Mode:       Min:    ", WHITE, BLACK);
    hsp_tft18_show_str_color(1, 2, "            Avg:    ", WHITE, BLACK);
    
    // window for TSL1401 waveform
    hsp_tft18_draw_frame(31, 64, 128, 64, BLUE);
    hsp_tft18_draw_block(32, 65, 128, 63, GRAY1);
}


// 蜂鸣器鸣叫函数
void beeper_beep(uint32_t duration_ms) {
    // 控制蜂鸣器鸣叫指定的毫秒数
	BUZZ_ON();
	delay_1ms(duration_ms);
	BUZZ_OFF();

}

// LED闪烁函数
void led_blink(uint32_t duration_ms) {
	// 控制LED闪烁指定的毫秒数 RGB交替闪烁

	if (led_state%3==0) //blue on
	{
		LED_B_ON(); 
		LED_R_OFF();
		LED_G_OFF();
	}
	else if (led_state%3==1) //red on
	{
		LED_B_OFF();
		LED_R_ON();
		LED_G_OFF();
	}
	else //green on
	{
		LED_B_OFF();
		LED_R_OFF();
		LED_G_ON();
		led_state = 0;
	}
	delay_1ms(duration_ms);
	led_state++;

}


void line_position_alert(int line_position) {
    uint32_t current_time = sys_tick_counter;
	static uint32_t last_blink_time = 0;
	
    // 判断SW1的状态，选择蜂鸣器或LED1指示
    bool use_beeper = !SW1();
	// bool use_beeper = 0; //led 供调试使用

    if (line_position < 0|| line_position >= CCD_PIXELS) {
        // 未检测到黑线
        if (current_time - last_blink_time > 1000) { // 超过1秒未检测到
            if (use_beeper) {
                beeper_beep(1000); // 蜂鸣器长鸣
            } else {
                led_blink(1000); // LED长亮
            }

        } else if (current_time - last_blink_time > 100) {
            if (use_beeper) {
                beeper_beep(100); // 蜂鸣器短鸣
            } else {
                led_blink(100); // LED短亮
            } 
        }
		// last_blink_time = current_time;
    } else if (line_position < 20 || line_position > 108) {
        // 黑线位置偏离中心
        if (current_time - last_blink_time > 200) {
            if (use_beeper) {
                beeper_beep(200); // 蜂鸣器短鸣
            } else {
                led_blink(200); // LED短亮
            }
            last_blink_time = current_time;
        }
    } else {
        // 黑线位置正常，不做操作
		LED1_OFF();
		BUZZ_OFF();
    }
}
