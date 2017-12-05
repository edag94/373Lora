#include "lora.h"


const uint8_t frame_size = 16;
const uint8_t burst_frame_size = 8;
const uint8_t modem_default[] = {0x72, 0x74, 0x00};
const double freq = 915.0;
const uint16_t preamble_len = 8;

volatile uint8_t buf_len;
volatile uint8_t rx_buf_valid;
uint8_t buf[RH_RF95_MAX_PAYLOAD_LEN];

uint8_t mode;

uint8_t this_address = RH_BROADCAST_ADDRESS;
uint8_t promiscuous = 0;

volatile uint8_t rx_header_to = RH_BROADCAST_ADDRESS;
volatile uint8_t rx_header_from = RH_BROADCAST_ADDRESS;
volatile uint8_t rx_header_id;
volatile uint8_t rx_header_flags;

uint8_t tx_header_to = RH_BROADCAST_ADDRESS;
uint8_t tx_header_from = RH_BROADCAST_ADDRESS;
uint8_t tx_header_id = 0x01;
uint8_t tx_header_flags = 0x00;

volatile int16_t last_rssi;

volatile uint16_t rx_bad;
volatile uint16_t rx_good;
volatile uint16_t tx_good;

volatile uint8_t cad;
unsigned int cad_timeout;

void handle_interrupt(void)
{
    // Read the interrupt register
    uint8_t irq_flags = read(RH_RF95_REG_12_IRQ_FLAGS);
    if (mode == MODE_RX && irq_flags & (RH_RF95_RX_TIMEOUT | RH_RF95_PAYLOAD_CRC_ERROR))
    {
    	rx_bad++;
    }
    else if (mode == MODE_RX && irq_flags & RH_RF95_RX_DONE)
    {
	// Have received a packet
		uint8_t len = read(RH_RF95_REG_13_RX_NB_BYTES);

		// Reset the fifo read ptr to the beginning of the packet
		write(RH_RF95_REG_0D_FIFO_ADDR_PTR, read(RH_RF95_REG_10_FIFO_RX_CURRENT_ADDR));
		burst_read(RH_RF95_REG_00_FIFO, buf, len);
		buf_len = len;
		write(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags

		// Remember the RSSI of this packet
		// this is according to the doc, but is it really correct?
		// weakest receiveable signals are reported RSSI at about -66
		last_rssi = read(RH_RF95_REG_1A_PKT_RSSI_VALUE) - 137;

		// We have received a message.
		validate_rx_buf();
		if (rx_buf_valid)
			set_mode_idle(); // Got one
    }
    else if (mode == MODE_TX && irq_flags & RH_RF95_TX_DONE)
    {
		tx_good++;
		set_mode_idle();
    }
    else if (mode == MODE_CAD && irq_flags & RH_RF95_CAD_DONE)
    {
        cad = irq_flags & RH_RF95_CAD_DETECTED;
        set_mode_idle();
    }

    write(RH_RF95_REG_12_IRQ_FLAGS, 0xff); // Clear all IRQ flags
}

void wait_available(void){
	while (available() == FALSE);
}

uint8_t wait_available_timeout(uint16_t timeout){
	unsigned long starttime = timeout * 1000;
	unsigned long counter = 0;
	while (counter++ < starttime){
		//printf("%d", counter);
		if (available() != FALSE){
			return TRUE;
		}
	}
	return FALSE;
}

uint8_t wait_packet_sent(uint16_t timeout){
	if (!timeout){
		while (mode == MODE_TX);
		return TRUE;
	}
	//unsigned long starttime = time(0);
	//while ((time(0) - starttime < timeout)){
	unsigned long starttime = timeout * 1000;
	unsigned long counter = 0;
	while (counter++ < starttime){
		if (mode != MODE_TX){
			return TRUE;
		}
	}
	return FALSE;
}

void set_promiscuous(uint8_t prom){
	promiscuous = prom;
}

void set_this_address(uint8_t addr){
	this_address = addr;
}

void set_header_to(uint8_t to){
	tx_header_to = to;
}

void set_header_from(uint8_t from){
	tx_header_from = from;
}

void set_header_id(uint8_t id){
	tx_header_id = id;
}

void set_header_flags(uint8_t set, uint8_t clear){
	tx_header_flags &= ~clear;
	tx_header_flags |= set;
}

// END RHGenericDriver code

uint8_t init(void){

	MSS_SPI_init(&g_mss_spi1);
	 MSS_SPI_config(frame_size);




	//MSS_GPIO_drive_inout(MSS_GPIO_10, MSS_GPIO_HIGH_Z);
	int i;
	write(RH_RF95_REG_01_OP_MODE, RH_RF95_LONG_RANGE_MODE);
//	    delay(10); // Wait for sleep mode to take over from say, CAD
	for (i=0;i<100000;i++);

	read(RH_RF95_REG_06_FRF_MSB);
	uint8_t read_result = read(RH_RF95_REG_01_OP_MODE);
	uint8_t comp = RH_RF95_MODE_SLEEP | RH_RF95_LONG_RANGE_MODE;
	    // Check we are in sleep mode, with LORA set
	if (read_result != comp)
	{
		printf("%d, %d\r\n", read_result, comp);
		//	Serial.println(spiRead(RH_RF95_REG_01_OP_MODE), HEX);
		return 1; // No device present?
	}

	// Sets up FIFO so transmit data starts at 0, receive starts at 128
	write(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
	write(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 128);

	set_mode_idle();

	set_modem_config(modem_default); // Radio default

	set_preamble_length(preamble_len);

	set_frequency(freq);

	// Lowish power
	set_tx_power(13, 0);

	return 0;
}

void set_mode_idle(void){
	if (mode != MODE_IDLE){
		write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_STDBY);
		mode = MODE_IDLE;
	}
}

void set_sleep_mode(void){
	if (mode != MODE_SLEEP){
		write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_SLEEP);
		mode = MODE_SLEEP;
	}
}

void set_mode_rx(void){
	if (mode != MODE_RX){
		write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_RXCONTINUOUS);
		write(RH_RF95_REG_40_DIO_MAPPING1, 0x00); // Interrupt on RxDone
		mode = MODE_RX;
	}
}

void set_mode_tx(void){
    if (mode != MODE_TX)
    {
		write(RH_RF95_REG_01_OP_MODE, RH_RF95_MODE_TX);
		write(RH_RF95_REG_40_DIO_MAPPING1, 0x40); // Interrupt on TxDone
		mode = MODE_TX;
    }
}

void set_modem_config(uint8_t *config){
	write(RH_RF95_REG_1D_MODEM_CONFIG1, config[0]);
	write(RH_RF95_REG_1E_MODEM_CONFIG2, config[1]);
	write(RH_RF95_REG_26_MODEM_CONFIG3, config[2]);
}

void set_preamble_length(uint16_t bytes){
	write(RH_RF95_REG_20_PREAMBLE_MSB, bytes >> 8);
	write(RH_RF95_REG_21_PREAMBLE_LSB, bytes & 0xff);
}

void set_frequency(double f){
	uint32_t frf = (f * 1000000.0) / RH_RF95_FSTEP;
	write(RH_RF95_REG_06_FRF_MSB, (frf >> 16) & 0xff);
	write(RH_RF95_REG_07_FRF_MID, (frf >> 8) & 0xff);
	write(RH_RF95_REG_08_FRF_LSB, frf & 0xff);
}

void set_tx_power(int8_t power, uint8_t useRFO)
{
    // Sigh, different behaviours depending on whther the module use PA_BOOST or the RFO pin
    // for the transmitter output
    if (useRFO){
		if (power > 14)
			power = 14;
		if (power < -1)
			power = -1;
		write(RH_RF95_REG_09_PA_CONFIG, RH_RF95_MAX_POWER | (power + 1));
    } else {
		if (power > 23)
			power = 23;
		if (power < 5)
			power = 5;

		// For RH_RF95_PA_DAC_ENABLE, manual says '+20dBm on PA_BOOST when OutputPower=0xf'
		// RH_RF95_PA_DAC_ENABLE actually adds about 3dBm to all power levels. We will use it
		// for 21, 22 and 23dBm
		if (power > 20) {
			write(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_ENABLE);
			power -= 3;
		} else {
			write(RH_RF95_REG_4D_PA_DAC, RH_RF95_PA_DAC_DISABLE);
		}

		// RFM95/96/97/98 does not have RFO pins connected to anything. Only PA_BOOST
		// pin is connected, so must use PA_BOOST
		// Pout = 2 + OutputPower.
		// The documentation is pretty confusing on this topic: PaSelect says the max power is 20dBm,
		// but OutputPower claims it would be 17dBm.
		// My measurements show 20dBm is correct
		write(RH_RF95_REG_09_PA_CONFIG, RH_RF95_PA_SELECT | (power-5));
	}
}

void validate_rx_buf(void){
	if (buf_len < 4){
		return;
	}
	rx_header_to = buf[0];
	rx_header_from = buf[1];
	rx_header_id = buf[2];
	rx_header_flags = buf[3];
	if (promiscuous ||
			rx_header_to == this_address ||
			rx_header_to == RH_BROADCAST_ADDRESS){
		rx_good++;
		rx_buf_valid = 1;
	}
}

uint8_t available(void){
	if (mode == MODE_TX){
		return FALSE;
	}
	set_mode_rx();
	return rx_buf_valid;
}

void clear_rx_buf(void){
	rx_buf_valid = FALSE;
	buf_len = 0;
}

uint8_t send(const uint8_t* data, uint8_t len){
	if (len > RH_RF95_MAX_MESSAGE_LEN){
		return FALSE;
	}

	wait_packet_sent(0);
	set_mode_idle();

	// Position at the beginning of the FIFO
	write(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);
	// The headers
	write(RH_RF95_REG_00_FIFO, tx_header_to);
	write(RH_RF95_REG_00_FIFO, tx_header_from);
	write(RH_RF95_REG_00_FIFO, tx_header_id);
	write(RH_RF95_REG_00_FIFO, tx_header_flags);
	// The message data
	burst_write(RH_RF95_REG_00_FIFO, data, len);
	write(RH_RF95_REG_22_PAYLOAD_LENGTH, len + RH_RF95_HEADER_LEN);

	set_mode_tx(); // Start the transmitter
	// when Tx is done, interruptHandler will fire and radio mode will return to STANDBY
}

uint8_t recv(uint8_t* in_buf, uint8_t* len){
    if (!available()){
    	return 0;
    }
	if (in_buf && len) {
		// Skip the 4 headers that are at the beginning of the rxBuf
		if (*len > buf_len - RH_RF95_HEADER_LEN)
			*len = buf_len - RH_RF95_HEADER_LEN;
		memcpy(in_buf, buf + RH_RF95_HEADER_LEN, *len);
    }
    clear_rx_buf(); // This message accepted and cleared
    return TRUE;
}

uint8_t read(uint8_t addr){
	uint8_t response;
	MSS_SPI_set_slave_select(&g_mss_spi1, MSS_SPI_SLAVE_0);
	MSS_GPIO_set_output(MSS_GPIO_0, 0);
	response = MSS_SPI_transfer_frame(&g_mss_spi1, addr << 8);
	MSS_GPIO_set_output(MSS_GPIO_0, 1);
	MSS_SPI_clear_slave_select(&g_mss_spi1, MSS_SPI_SLAVE_0);
	return response;
}

void write(uint8_t addr, uint8_t data){
	uint16_t cmd = (1 << 15) | (addr << 8) | data;
	MSS_SPI_set_slave_select(&g_mss_spi1, MSS_SPI_SLAVE_0);
	MSS_GPIO_set_output(MSS_GPIO_0, 0);
	printf("%d", cmd);
	MSS_SPI_transfer_frame( &g_mss_spi1, cmd);
	MSS_GPIO_set_output(MSS_GPIO_0, 1);
	MSS_SPI_clear_slave_select(&g_mss_spi1, MSS_SPI_SLAVE_0);
}

uint8_t burst_read(uint8_t addr, uint8_t* res, uint8_t len){
	 MSS_SPI_config(burst_frame_size);
  uint8_t status = 0;
  MSS_SPI_set_slave_select(&g_mss_spi1, MSS_SPI_SLAVE_0);
  MSS_GPIO_set_output(MSS_GPIO_0, 0);
  status = MSS_SPI_transfer_frame(&g_mss_spi1, addr);
  int i;
  for (i = 0; i < len; ++i){
    res[i] = MSS_SPI_transfer_frame(&g_mss_spi1, 0);
  }
  MSS_GPIO_set_output(MSS_GPIO_0, 1);
  MSS_SPI_clear_slave_select(&g_mss_spi1, MSS_SPI_SLAVE_0);
  MSS_SPI_config(frame_size);
  return status;
}

uint8_t burst_write(uint8_t addr, uint8_t* src, uint8_t len){
	 MSS_SPI_config(burst_frame_size);
  uint8_t status = 0;
  MSS_SPI_set_slave_select(&g_mss_spi1, MSS_SPI_SLAVE_0);
  MSS_GPIO_set_output(MSS_GPIO_0, 0);
  status = MSS_SPI_transfer_frame(&g_mss_spi1, (1 << 7) | addr);
  int i;
  for (i = 0; i < len; ++i){
    MSS_SPI_transfer_frame(&g_mss_spi1, src[i]);
  }
  MSS_GPIO_set_output(MSS_GPIO_0, 1);
  MSS_SPI_clear_slave_select(&g_mss_spi1, MSS_SPI_SLAVE_0);
  MSS_SPI_config(frame_size);
  return status;
}

void MSS_SPI_config(uint16_t frame_size){
MSS_SPI_configure_master_mode
	(
		&g_mss_spi1,
		MSS_SPI_SLAVE_0,
		MSS_SPI_MODE0,
		MSS_SPI_PCLK_DIV_256,
		frame_size
	);
}
