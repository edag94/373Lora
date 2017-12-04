#include <stdio.h>
#include <inttypes.h>

#include <drivers/mss_gpio/mss_gpio.h>
//#include <>

#include "LoRa_hw_platform.h"



__attribute__ ((interrupt)) void GPIO0_IRQHandler( void ){
	MSS_GPIO_clear_irq(MSS_GPIO_1);
	handle_interrupt();
}

// Main program
int main()
{
	MSS_GPIO_init();

	//for droping the select low to initiate the SPI transaction
	MSS_GPIO_config( MSS_GPIO_0, MSS_GPIO_OUTPUT_MODE);
	MSS_GPIO_set_output(MSS_GPIO_0, 1);

	//this is the interrupt
	MSS_GPIO_config( MSS_GPIO_1, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
	MSS_GPIO_enable_irq(MSS_GPIO_1);
	if (!init()){
		return(-1);
	}


	uint8_t send_buf[] = {0x01, 0x03, 0x05, 0x07};

	write(REG_0D_FIFO_ADDR_PTR, 0);
	burst_write(REG_00_FIFO, send_buf, 4);

	int i;
	for (i = 0; i < 100000; ++i);

	write(REG_0D_FIFO_ADDR_PTR, 0);
	uint8_t read_buf[4];
	burst_read(REG_00_FIFO, read_buf, 4);

	client_ex_setup();

	return(0);
}



