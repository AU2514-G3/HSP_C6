#include "hsp_liball.h"
#include "Lab1.h"

extern uint8_t RES_value;
extern uint32_t sys_tick_counter;

// collect rotary encoder switch signal by polling
void Lab1_res_polling()
{
	uint8_t state_pha, state_phb;
	uint8_t state_pha_t, state_phb_t;
	uint8_t pulse_counter = 0;
	
	state_pha = PHA2(),			state_phb = PHB2();
	state_pha_t = state_pha,	state_phb_t = state_phb;
	
	sys_tick_counter = 0;
	while(1)
	{
		state_pha = PHA2();
		state_phb = PHB2();
		if(state_pha != state_pha_t)	// PHA2 changed
		{
			if(SET == state_pha)
			{
				if(RESET == state_phb)	// 实测逻辑关系
				{
					pulse_counter++;
				}
				else if(pulse_counter>0)
				{
					pulse_counter--;
				}
			}
			state_pha_t = state_pha;
			state_phb_t = state_phb;
		}
		hsp_cat9555_seg7_decimal(pulse_counter);
		
		if (sys_tick_counter >= 1000)
		{
			BUZZ_ON();
			if (sys_tick_counter >= 1020)
			{
				BUZZ_OFF();
				sys_tick_counter = 20;
			}
		}
	}
}

// collect rotary encoder switch signal by interrupt
void Lab1_res_interrupt()
{
	uint8_t pulse_counter = 0;

    /* PHA2/PB14 interrupt enable */
    rcu_periph_clock_enable(RCU_SYSCFG);
    nvic_irq_enable(EXTI10_15_IRQn, 2U, 0U);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOB, EXTI_SOURCE_PIN14);
    exti_init(EXTI_14, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_flag_clear(EXTI_14);

	pulse_counter = RES_value;
	hsp_cat9555_seg7_decimal(pulse_counter);
	while(1)
	{
		if(RES_value != pulse_counter)
		{
			pulse_counter = RES_value;
			hsp_cat9555_seg7_decimal(pulse_counter);
		}
	}
}