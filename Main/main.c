/*!
    \file    main.c
    \brief   running led

    \version 2023-06-25, V3.1.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2023, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include <stdio.h>
#include <stdint.h>
#include <LowLevelIOInterface.h>
#include "gd32f4xx.h"
#include "systick.h"
#include "hsp_liball.h"

volatile uint32_t sys_tick_counter=0;
volatile uint8_t RES_value;
uint8_t task_id=0;       // TaskID set by 4-bit DIP switch
uint8_t menu_id=0;       // MenuItem ID returned by Menu_Loop()

#pragma location = 0X20030000
uint8_t image_raw[22560];
//__no_init uint8_t image_raw[22560][22560];
//__attribute__((aligned(32))) uint8_t image_raw[22560][22560];

/* HSP board initialize */
void board_init()
{
	/* systick timer 1000Hz interrupts, ISR: SysTick_Handler() */
    systick_config();

	/* GPIO interface such as button, switch, led, buzz, etc */
	hsp_gpio_init();
	
	/* SPI interface for LCD */
	hsp_spi_init();
	//LCD_BL_ON();
	
	/* I2C interface for CAT9555 */
	hsp_cat9555_init();
	
	/* UART interface for OpenSDA, wireless module, OpenMV, K210 */
	//hsp_uart_init();
	
	/* PIT periodical interrupt timer, for sensor refresh or PID algorithm */
	//hsp_pit_config();
	
	/* optical encoder pulse counter for motor speed feedback */
	hsp_counter_init();
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
  	uint16_t i;
	
	board_init();
	
	/* Exercise: test printf re-direction function __dwrite */
	//printf("\nHello SJTU HSP!\n\r");
	
    while(1) {
		// organize exercises, labs and project properly into a menu 
		if (!(sys_tick_counter % 20))
		{
			sys_tick_counter = 0;
			// LED_R_TOGGLE();
			i = timer_counter_read(TIMER3);
			i++;
			i |= 0b00000001;
			//hsp_tft18_show_uint16_color(0, 1, i, WHITE, BLACK);
			printf("counter value: %d\n\r", i);
		}


		menu_id = hsp_menu_loop();

		switch(menu_id)
		{
			case 0U:		// 1.Welcome HSP
			  	Ex3_3_bitmap();
				break;
			case 1U:		// 2.Lab1: HMI Basic IOs
				hsp_demo_hmi();
				break;
			case 2U:		// 3.Lab2: MMA8451
				Ex2_4_mems();
				break;
			case 3U:		// 4.Lab3: TSL1401 Linear CCD
				Lab3_test();
				break;
			case 4U:		// 5.Lab4: Motor and R/C servo
				// Ex6_2_servo_manual();
				Ex6_3_motor_manual();

				break;
			case 5U:		// 6.Project: Line Following Robot
				Project_LFR();
				break;
			case 6U:		// 7.RTC: Real Time Clock
				hsp_rtc_demo();		// **debug pending**
				break;
			case 7U:		// 8:System information
				hsp_demo_hmi();
				break;
		}
		
		while(!S3());
    }
}
	

/* Code for exercise reference */
//Ex0_gif_poweron();
//Ex1_1_led();
//Ex1_2_buzz();
//Ex1_3_led_switch();
//Ex2_1_seg7();
//Ex2_2_ledbar();
//Ex2_3_segbarmux();
//Ex2_4_mems();
//Ex3_1_tft18_text();
//Ex3_2_tft18_menu();
//Ex3_3_bitmap();
//Ex4_1_adc0();
//Ex4_2_adc1();
//Ex4_3_adc2();
//Ex4_4_adc01();
//Ex4_5_adc012();
//Ex5_2_uart2_opensda();
//Ex5_3_uart5_wireless();
//Ex6_1_servo_sweep();
//Ex6_2_servo_manual();
//Ex6_3_motor_manual();
//Ex7_1_dcam();
//Ex7_2_edge();
//Ex8_1_rtc();
//Lab1_res_polling();
//Lab1_res_interrupt();
//Lab3_test();
//Lab4_test();
//Project_LFR();
//		Lab3_seekfree();
//		Ex3_3_bitmap();
//		Project_LFR();
//		Ex6_3_motor_manual();
//		Ex7_1_dcam();
//			hsp_demo_hmi();
//			Ex4_1_adc0();
//			Ex2_2_ledbar();
//			Ex2_4_mems();
//			Ex3_4_bitmap2();
//			delay_1ms(2000);
//			testAll();
//			hsp_rtc_demo();
//			//Ex8_1_rtc();
//			hsp_demo_hmi();
		// if (!(sys_tick_counter % 20))
		// {
		// 	sys_tick_counter = 0;
		// 	LED_R_TOGGLE();
		// 	//i = timer_counter_read(TIMER3);
		// 	i++;
		// 	i |= 0b00000001;
		// 	//hsp_tft18_show_uint16_color(0, 1, i, WHITE, BLACK);
		// 	printf("counter value: %d\n\r", i);
		// }
