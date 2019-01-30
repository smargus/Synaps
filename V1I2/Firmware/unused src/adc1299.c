/*
 * adc1299.c
 *
 *  Created on: 3 окт. 2018 г.
 *      Author: ai
 */

#ifdef __cplusplus
extern "C" {
#endif

#include "adc1299.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "gpio.h"
#include "spi.h"
#include "usb.h"

extern osSemaphoreId adcDataReadySemathoreHandle;

uint32_t ads1299_check_device_id(void);

void adcTask(void const * argument)
{
    ads1299_device_init(1);

    printf("Checking ADS1299 device ID...\r\n");
    if ( 0 != ads1299_check_device_id())
	printf("Invalid ID. Possible SPI error.\r\n");
    else
	printf("Device ID verified.\r\n");

    ads1299_send_byte(ADS1299_OPC_RDATAC);
    ads1299_soft_start_conversion();

    volatile uint32_t sample_idx = 0;
    volatile uint32_t status = 0;
    volatile int32_t data_array[16][8];

    for(;;)
    {
	osSemaphoreWait(adcDataReadySemathoreHandle,1000);
	ads1299_rdata32_generic(sample_idx, &status, data_array);
	//ads1299_soft_start_conversion();
	printf("Status:%8X %8li|%8li|%8li|%8li|%8li|%8li|%8li|%8li\r\n",status,data_array[0][0],data_array[0][1],data_array[0][2],data_array[0][3],data_array[0][4],data_array[0][5],data_array[0][6],data_array[0][7]);
	//osDelay(100);
    }
}


/* SYSTEM CONTROL FUNCTIONS **********************************************************************************************************************/

uint32_t ads1299_check_device_id(void)
{
	uint8_t id;

	ads1299_rreg(ADS1299_REGADDR_ID, &id);
	/* ADS1299 should return ID of 0bXXXX1110 */
	if ( (id & 0xF) != ADS1299_DEVICE_ID ) {
		return 1;
	}
	else {
		return 0;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//    if (GPIO_Pin == ADC_DRDY_Pin)
//    {
	osSemaphoreRelease(adcDataReadySemathoreHandle);
//    }

}

ads1299_error_t	ads1299_device_init(uint8_t init_regs)
{
    /* Power cycle ADS1299 */
    HAL_GPIO_WritePin(ADC_PWDN_GPIO_Port, ADC_PWDN_Pin, GPIO_PIN_RESET);

    //delay_us(20);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    HAL_GPIO_WritePin(ADC_PWDN_GPIO_Port, ADC_PWDN_Pin, GPIO_PIN_SET);

    /* Allow oscillator warm-up */
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    //delay_ms(1000);

    /* Tell chip to exit continuous data mode */
    ads1299_send_byte(ADS1299_OPC_SDATAC);
    /* Stop taking conversions; apparently not done automatically */
    ads1299_send_byte(ADS1299_OPC_STOP);

    if (init_regs)
    {
	/* Write to GPIO register, set all pins to driven-low output */
	ads1299_wreg( ADS1299_REGADDR_GPIO, ADS1299_REG_GPIO_GPIOC4_OUTPUT |
	ADS1299_REG_GPIO_GPIOD4_LOW    |
	ADS1299_REG_GPIO_GPIOC3_OUTPUT |
	ADS1299_REG_GPIO_GPIOD3_LOW    |
	ADS1299_REG_GPIO_GPIOC2_OUTPUT |
	ADS1299_REG_GPIO_GPIOD2_LOW    |
	ADS1299_REG_GPIO_GPIOC1_OUTPUT |
	ADS1299_REG_GPIO_GPIOD1_LOW    );

	/* Write to CONFIG1, set data rate to 250 Hz */
	ads1299_wreg( ADS1299_REGADDR_CONFIG1, ADS1299_REG_CONFIG1_RESERVED_VALUE |
	ADS1299_REG_CONFIG1_FMOD_DIV_BY_4096);
	/* Write to CONFIG2 register, generate test signal internally */
	ads1299_wreg( ADS1299_REGADDR_CONFIG2, ADS1299_REG_CONFIG2_RESERVED_VALUE |
	ADS1299_REG_CONFIG2_CAL_INT        |
	ADS1299_REG_CONFIG2_CAL_PULSE_FCLK_DIV_2_21);

	/* Write to CONFIG3, enable internal reference buffer, bias internally generated, bias buffer enabled */
	ads1299_wreg( ADS1299_REGADDR_CONFIG3, ADS1299_REG_CONFIG3_REFBUF_ENABLED |
	ADS1299_REG_CONFIG3_BIASREF_INT    |
	ADS1299_REG_CONFIG3_BIASBUF_ENABLED);
	/* Reference settling time */
	osDelay(150);
	//delay_ms(150);

	/* Write to CH1 settings register, set as normal input, gain 24 */
	ads1299_wreg( ADS1299_REGADDR_CH1SET, ADS1299_REG_CHNSET_CHANNEL_ON			|
	ADS1299_REG_CHNSET_GAIN_24			|
	ADS1299_REG_CHNSET_SRB2_DISCONNECTED	|
	ADS1299_REG_CHNSET_NORMAL_ELECTRODE);
	/* Write to CH2 settings register, set as normal input, gain 24 */
	ads1299_wreg( ADS1299_REGADDR_CH2SET, ADS1299_REG_CHNSET_CHANNEL_ON			|
	ADS1299_REG_CHNSET_GAIN_24			|
	ADS1299_REG_CHNSET_SRB2_DISCONNECTED	|
	ADS1299_REG_CHNSET_NORMAL_ELECTRODE);
	/* Write to CH3 settings register, set as normal input, gain 24 */
	ads1299_wreg( ADS1299_REGADDR_CH3SET, ADS1299_REG_CHNSET_CHANNEL_ON			|
	ADS1299_REG_CHNSET_GAIN_24			|
	ADS1299_REG_CHNSET_SRB2_DISCONNECTED	|
	ADS1299_REG_CHNSET_NORMAL_ELECTRODE);
	/* Write to CH4 settings register, set as normal input, gain 24 */
	ads1299_wreg( ADS1299_REGADDR_CH4SET, ADS1299_REG_CHNSET_CHANNEL_ON			|
	ADS1299_REG_CHNSET_GAIN_24			|
	ADS1299_REG_CHNSET_SRB2_DISCONNECTED	|
	ADS1299_REG_CHNSET_NORMAL_ELECTRODE);
	/* Write to CH5 settings register, set as normal input, gain 24 */
	ads1299_wreg( ADS1299_REGADDR_CH5SET, ADS1299_REG_CHNSET_CHANNEL_ON			|
	ADS1299_REG_CHNSET_GAIN_24			|
	ADS1299_REG_CHNSET_SRB2_DISCONNECTED	|
	ADS1299_REG_CHNSET_NORMAL_ELECTRODE);
	/* Write to CH6 settings register, set as normal input, gain 24 */
	ads1299_wreg( ADS1299_REGADDR_CH6SET, ADS1299_REG_CHNSET_CHANNEL_ON			|
	ADS1299_REG_CHNSET_GAIN_24			|
	ADS1299_REG_CHNSET_SRB2_DISCONNECTED	|
	ADS1299_REG_CHNSET_NORMAL_ELECTRODE);
	/* Write to CH5 settings register, set as normal input, gain 24 */
	ads1299_wreg( ADS1299_REGADDR_CH7SET, ADS1299_REG_CHNSET_CHANNEL_ON			|
	ADS1299_REG_CHNSET_GAIN_24			|
	ADS1299_REG_CHNSET_SRB2_DISCONNECTED	|
	ADS1299_REG_CHNSET_NORMAL_ELECTRODE);
	/* Write to CH6 settings register, set as normal input, gain 24 */
	ads1299_wreg( ADS1299_REGADDR_CH8SET, ADS1299_REG_CHNSET_CHANNEL_ON			|
	ADS1299_REG_CHNSET_GAIN_24			|
	ADS1299_REG_CHNSET_SRB2_DISCONNECTED	|
	ADS1299_REG_CHNSET_NORMAL_ELECTRODE);

	/* Write to MISC1 register, SRB1 on (ref electrode) */
	ads1299_wreg( ADS1299_REGADDR_MISC1, ADS1299_REG_MISC1_SRB1_ON);
    }
    return ADS1299_STATUS_OK;
}



/* REGISTER READ/WRITE FUNCTIONS *****************************************************************************************************************/

ads1299_error_t ads1299_rreg(uint8_t reg_addr, uint8_t* read_reg_val_ptr)
{
    //uint16_t read_data;
    uint8_t txbuf [3] = {0};
    uint8_t rxbuf [3] = {0};

    txbuf[0] = ADS1299_OPC_RREG | reg_addr;

    //ads1299_cs_select();

    HAL_SPI_TransmitReceive(&hspi3,txbuf,rxbuf,3,1000);
    //osDelay(1);
    //HAL_SPI_Receive (&hspi3,&rxbuf,1,1000);

    //ads1299_cs_deselect();

    *read_reg_val_ptr = rxbuf[2];

//    spi_selectChip(SPI_ADDRESS, chip_select);
//
//    /* First byte: read command for specified register */
//    ads1299_send_byte_no_cs(ADS1299_OPC_RREG | reg_addr);
//
//    /* Second byte: Read only 1 register (send n-1, where n is number of registers to read) */
//    ads1299_send_byte_no_cs(0x00);
//
//    /* Dummy byte to clock in data */
//    ads1299_send_byte_no_cs(DUMMY_BYTE);
//
//    delay_us(10);
//    spi_unselectChip(SPI_ADDRESS, chip_select);
//
//    /* Read SPI RX register */
//    read_data = spi_get(SPI_ADDRESS);
//    *read_reg_val_ptr = (uint8_t) read_data;

    return ADS1299_STATUS_OK;
}

ads1299_error_t ads1299_wreg(uint8_t reg_addr, uint8_t reg_val_to_write)
{
//    spi_selectChip(SPI_ADDRESS, chip_select);
//
//    /* First byte: write command for specified register */
//    ads1299_send_byte_no_cs(ADS1299_OPC_WREG | reg_addr);
//
//    /* Second byte: number of registers to write (1) */
//    ads1299_send_byte_no_cs(0x00);
//
//    /* Third byte: write register value */
//    ads1299_send_byte_no_cs(reg_val_to_write);
//
//    spi_unselectChip(SPI_ADDRESS, chip_select);

    //uint16_t read_data;
    uint8_t txbuf [3];

    txbuf[0] = ADS1299_OPC_WREG | reg_addr;

    txbuf[1] = 0x00;

    txbuf[2] = reg_val_to_write;

//    ads1299_cs_select();

    HAL_SPI_Transmit(&hspi3,txbuf,3,1000);

//    ads1299_cs_deselect();

    return ADS1299_STATUS_OK;
}

/* DATA RETRIEVAL FUNCTIONS **********************************************************************************************************************/
//
//ads1299_error_t ads1299_rdata32_packet(uint8_t chip_select, volatile uint32_t sample_idx, bboard_data32bit_packet_t* packet_ptr)
//{
//    volatile uint8_t channel_idx;
//    union {
//	    uint32_t raw;
//	    uint8_t status[4];
//    } __attribute__((packed)) statustemp;
//    union {
//	    int32_t raw;
//	    uint8_t data[4];
//    } __attribute__((packed)) sigtemp;
//    statustemp.raw = 0;
//    sigtemp.raw = 0;
//
//    /* Begin SPI comms */
////    spi_selectChip(SPI_ADDRESS, chip_select);
//
//    /* Function assumes we've already sent RDATA command or are in RDATAC mode */
//
//    /* Read in status word first (24 bits) */
////    spi_read_packet(SPI_ADDRESS, &statustemp.status[1], 3);
////    packet_ptr->eegstatus = statustemp.raw;
////
////    /* Begin reading in data */
////    for (channel_idx = 0; channel_idx < MAX_EEG_CHANNELS; channel_idx++)
////    {
////	    spi_read_packet(SPI_ADDRESS, &sigtemp.data[1], 3);
////	    packet_ptr->eegdata[sample_idx][channel_idx] = SIGN_EXT_24(sigtemp.raw);
////    }
////
////    spi_unselectChip(SPI_ADDRESS, chip_select);
//
//
//
//    return ADS1299_STATUS_OK;
//}
//

ads1299_error_t ads1299_rdata24_packet(volatile uint32_t sample_idx, data24bit_packet_t* packet_ptr)
{
    volatile uint8_t channel_idx;

    uint8_t temprx[27]={0};

    /* Function assumes we've already sent RDATA command or are in RDATAC mode */

//    ads1299_cs_select();

    HAL_SPI_Receive(&hspi3,temprx,27,1000);

//    ads1299_cs_deselect();

    packet_ptr->adcstatus[0] = temprx[0];
    packet_ptr->adcstatus[1] = temprx[1];
    packet_ptr->adcstatus[2] = temprx[2];

    /* Begin reading in data */
    for (channel_idx = 0; channel_idx < 8; channel_idx++)
    {
	packet_ptr->adcdata[sample_idx][channel_idx][0] = temprx[channel_idx * 3 + 3];
	packet_ptr->adcdata[sample_idx][channel_idx][1] = temprx[channel_idx * 3 + 4];
	packet_ptr->adcdata[sample_idx][channel_idx][2] = temprx[channel_idx * 3 + 5];
    }

    return ADS1299_STATUS_OK;
}

ads1299_error_t ads1299_rdata24_generic(volatile uint32_t sample_idx, volatile uint8_t status_array[], volatile uint8_t data_array[][8][3])
{
    volatile uint8_t channel_idx;

    /* Function assumes we've already sent RDATA command or are in RDATAC mode */

    /* Read in status word first (24 bits) */

    uint8_t temprx[27]={0};

    /* Begin reading in data */

//    ads1299_cs_select();

    HAL_SPI_Receive(&hspi3,temprx,27,1000);

//    ads1299_cs_deselect();

    status_array[0] = temprx[0];
    status_array[1] = temprx[1];
    status_array[2] = temprx[2];

    for (channel_idx = 0; channel_idx < 8; channel_idx++)
    {
	    data_array[sample_idx][channel_idx][0] = temprx[channel_idx * 3 + 3];
	    data_array[sample_idx][channel_idx][1] = temprx[channel_idx * 3 + 4];
	    data_array[sample_idx][channel_idx][2] = temprx[channel_idx * 3 + 5];
    }

    return ADS1299_STATUS_OK;
}

ads1299_error_t ads1299_rdata32_generic(volatile uint32_t sample_idx, volatile uint32_t * status, volatile int32_t data_array[16][8])
{
    uint8_t temprx[27]={0};

    volatile uint8_t channel_idx;
    union {
	    uint32_t raw;
	    uint8_t status[4];
    } __attribute__((packed)) statustemp;
    union {
	    int32_t raw;
	    uint8_t data[4];
    } __attribute__((packed)) sigtemp;
    statustemp.raw = 0;
    sigtemp.raw = 0;

    /* Function assumes we've already sent RDATA command or are in RDATAC mode */

    /* Read in status word first (24 bits) */

    /* Begin reading in data */
    ads1299_cs_select();

    HAL_SPI_Receive(&hspi3,temprx,27,1000);

    ads1299_cs_deselect();

    statustemp.status[0] = temprx[0];
    statustemp.status[1] = temprx[1];
    statustemp.status[2] = temprx[2];
    statustemp.status[3] = 0;

    *status = statustemp.raw;

    /* Begin reading in data */
    for (channel_idx = 0; channel_idx < 8; channel_idx++)
    {
	sigtemp.data[0] = temprx[channel_idx * 3 + 3];
	sigtemp.data[1] = temprx[channel_idx * 3 + 4];
	sigtemp.data[2] = temprx[channel_idx * 3 + 5];
	sigtemp.data[3] = 0;

	data_array[sample_idx][channel_idx] = SIGN_EXT_24(sigtemp.raw);
    }

    return ADS1299_STATUS_OK;
}



#ifdef __cplusplus
}
#endif
