// https://blog.csdn.net/qq_44691051/article/details/113767720
// https://blog.csdn.net/qq_31251431/article/details/105862085
// https://www.cnblogs.com/Mr-Wangblogs/p/9060417.html


#include "HSP_TSL1401.h"

// ADC_IDATA0(ADC1): CCD1 - PC2
// ADC_IDATA1(ADC1): CCD2 - PC3
void hsp_ccd_init()
{
	hsp_adc_init();
	hsp_adc1_config();
	hsp_adc2_config();	// test on EM-PF3
}

void hsp_ccd_flush(void)
{
    uint8_t index = 0;

	CCD1_SI_HIGH;
	CCD2_SI_HIGH;
    hsp_ccd_delay();
	
    CCD1_CLK_HIGH;
    CCD2_CLK_HIGH;
    hsp_ccd_delay();
	
    CCD1_SI_LOW;
    CCD2_SI_LOW;
    hsp_ccd_delay();
	
    CCD1_CLK_LOW;
    CCD2_CLK_LOW;

    for(index=0; index<128; index++)
//    for(index=0; index<127; index++)
    {
        hsp_ccd_delay();
        hsp_ccd_delay();
		CCD1_CLK_HIGH;
		CCD2_CLK_HIGH;
        hsp_ccd_delay();
        hsp_ccd_delay();
		CCD1_CLK_LOW;
		CCD2_CLK_LOW;
    }
//	hsp_ccd_delay();
//	hsp_ccd_delay();
//	CCD1_CLK_HIGH;
//	CCD2_CLK_HIGH;
//	hsp_ccd_delay();
//	hsp_ccd_delay();
//	CCD1_CLK_LOW;
//	CCD2_CLK_LOW;
}

void hsp_ccd_snapshot(ccd_t linear_array) 
{
    uint8_t index=0;

    // flush previously integrated frame first
    hsp_ccd_flush();
    // wait for TSL1401 to integrate new frame
	// control exposure time by delay
    delay_1ms(10); //曝光时间

	CCD1_SI_HIGH;
	CCD2_SI_HIGH;
    hsp_ccd_delay();

	CCD1_CLK_HIGH;
    CCD2_CLK_HIGH;
    hsp_ccd_delay();
    
	CCD1_SI_LOW;
    CCD2_SI_LOW;
    hsp_ccd_delay();
	
    CCD1_CLK_LOW;
    CCD2_CLK_LOW;

    for(index=0; index<128; index++)
    {
		hsp_ccd_delay();

		/* ADC software trigger enable */
        adc_software_trigger_enable(ADC1, ADC_INSERTED_CHANNEL);
		/* wait for EOC */
		while(!adc_flag_get(ADC1, ADC_FLAG_EOC));
		// use CCD#1
		//linear_array[index] = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_0);
		// use CCD#2
		linear_array[index] = adc_inserted_data_read(ADC1, ADC_INSERTED_CHANNEL_1);

		/* ADC software trigger enable */
        //adc_software_trigger_enable(ADC2, ADC_INSERTED_CHANNEL);
		/* wait for EOC */
		//while(!adc_flag_get(ADC2, ADC_FLAG_EOC));
		// use EM-PF3
		//linear_array[index] = adc_inserted_data_read(ADC2, ADC_INSERTED_CHANNEL_0);
		
		// clock pulse to read next pixel output
		CCD1_CLK_HIGH;
		CCD2_CLK_HIGH;
		hsp_ccd_delay();
		CCD1_CLK_LOW;
		CCD2_CLK_LOW;
    }

    // 129th pulse to terminate output of 128th pixel
	CCD1_CLK_HIGH;
	CCD2_CLK_HIGH;
	hsp_ccd_delay();
	CCD1_CLK_LOW;
	CCD2_CLK_LOW;
}

void hsp_ccd_delay(void)
{
	uint16_t count;
	
	count = 160;
	while(count--);
}
