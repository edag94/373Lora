#include <drivers/mss_gpio/mss_gpio.h>
#include <inttypes.h>
#include "lora.h"


__attribute__ ((interrupt)) void GPIO1_IRQHandler( void ){
	MSS_GPIO_clear_irq(MSS_GPIO_1);
	handle_interrupt();
}

void MSS_setup(void){
	MSS_GPIO_init();
		MSS_GPIO_config( MSS_GPIO_0, MSS_GPIO_OUTPUT_MODE);
		MSS_GPIO_config( MSS_GPIO_1, MSS_GPIO_INPUT_MODE | MSS_GPIO_IRQ_EDGE_POSITIVE );
		MSS_GPIO_enable_irq(MSS_GPIO_1);
		MSS_GPIO_set_output(MSS_GPIO_0, 1);
}

// Main program
int main()
{

	MSS_setup();
	int init_res = init();

	uint8_t send_buf[] = {0x01, 0x03, 0x05, 0x07};

	write(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);
	burst_write(RH_RF95_REG_00_FIFO, send_buf, 4);

	int i;
	for (i = 0; i < 100000; ++i);

	write(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);
	uint8_t read_buf[4];
	burst_read(RH_RF95_REG_00_FIFO, read_buf, 4);
	client_ex_setup();

	//read_addr(RegFifoTxBaseAddr);

	return(0);

}


