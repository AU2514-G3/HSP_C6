#ifndef _LAB3_H
#define _LAB3_H

#include "hsp_adc.h"
#include "hsp_gpio.h"
#include "hsp_spi.h"
#include "hsp_uart.h"
#include "hsp_timer.h"
#include "HSP_TFT18.h"
#include "HSP_TSL1401.h"
#include "HSP_MOTOR.h"

void Lab3_test(void);
void Lab3_seekfree(void);
void hsp_ccd_show(ccd_t data,int line_position);
void hsp_demo_frame_ccd(void);

#endif