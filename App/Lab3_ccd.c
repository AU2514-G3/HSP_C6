// https://blog.csdn.net/u014545515/article/details/38363511/
// https://www.renrendoc.com/paper/161650011.html


#include "HSP_TSL1401.h"
#include "Lab3.h"

ccd_t ccd_data_raw, ccd_data_old;
uint8_t CCD2PC[260];	// data to be sent to PC using seekfree protocol
uint16_t max_v, min_v;				// max/min value of the linear array
uint8_t max_v_index, min_v_index;	// array index of the max/min_v
int16_t max_dv, min_dv;				// max/min value of the linear array delta
uint8_t max_dv_index, min_dv_index;	// array index of the max/min_dv
int16_t delta_v[124];				// delta v of the linear array

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
		hsp_ccd_show(ccd_data_raw);
		
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
		hsp_ccd_show(ccd_data_raw);

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

// draw LinearCCD waveform by pixels: [32,128] ~ [160,64]
void hsp_ccd_show(ccd_t data)
{
    uint8_t i=0;

    for(i=0; i<CCDPIXEL; i++)
	{
        hsp_tft18_draw_pixel(32+i, 128-(ccd_data_old[i]>>6), GRAY1);
        hsp_tft18_draw_pixel(32+i, 128-(data[i]>>6), BLUE);			// data/64 : 0~4095 -> 0~64
        ccd_data_old[i] = data[i];
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
