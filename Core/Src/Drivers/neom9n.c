#include "main.h"
#include "neom9n.h"

//INTERFACE DESCRIPTION:
//https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf?utm_content=UBX-21022436

//todo make this actually print something
static void print_HAL_Status(HAL_StatusTypeDef status) {
	switch (status) {
	case HAL_OK:
		break;
	case HAL_ERROR:
		break;
	case HAL_BUSY:
		break;
	case HAL_TIMEOUT:
		break;
	}
}

static void cs_low(NeoGPSConfig_t *config) {
	HAL_GPIO_WritePin(config->cs_pin_port, config->cs_pin, GPIO_PIN_RESET);
}

static void cs_high(NeoGPSConfig_t *config) {
	HAL_GPIO_WritePin(config->cs_pin_port, config->cs_pin, GPIO_PIN_SET);
}

void calculateChecksum(UBX_Packet_t *packet, uint8_t *tx, uint16_t tx_size) {
	packet->checksumA = 0;
	packet->checksumB = 0;

	for (int i = 0; i < tx_size; i++) {
		packet->checksumA += tx[i];
		packet->checksumB += packet->checksumA;
	}
}



//ACK MESSAGE IS IN THE FORMAT HEADER | CLASS (0X05) | ID (0X01 FOR ACK 0X00 FOR NOT ACK) | len | payload | checksum
void waitForAck(UBX_Packet_t  *outgoing, uint16_t max_wait) {

}

/*
 * To check if the device is connected,
 * we check the port settings of some
 * port to see if we get a valid result.
 */
void isConnected(NeoGPSConfig_t *config, uint16_t max_wait) {
	UBX_Packet_t packet;
	packet.class = UBX_CLASS_CFG; //configuration
	packet.id = UBX_CFG_PRT; 	  //Polls the configuration for one I/O port
	packet.length = 1;

	//create the payload
	uint8_t payload = PORT_ID_SPI;

	packet.payload = &payload;
}

void sendSPICommand(NeoGPSConfig_t *config, UBX_Packet_t *outgoing, uint8_t *rx_buf, uint16_t max_wait) {
	//start with header bytes
	//determine size of
	uint16_t tx_size = UBX_PACKET_HEADER_SIZE +
			   outgoing->payload_length +
			   UBX_PACKET_FOOTER_SIZE;

	uint8_t tx[tx_size];

	tx[0] = UBX_PSYNC_1;
	tx[1] = UBX_PSYNC_2;

	tx[2] = outgoing->class;
	tx[3] = outgoing->id;
	tx[4] = outgoing->length & 0xFF; //LSB OF LENGTH,
	tx[5] = outgoing->length >> 8;   //MSB OF LENGTH. LENGTH IS LSB FIRST PER DATASHEET

	for (int i = 0; i < outgoing->payload_length; i++) {
		tx[UBX_PACKET_HEADER_SIZE + i] = outgoing->payload[i];
	}

	int footer_start = UBX_PACKET_HEADER_SIZE + outgoing->payload_length;

	tx[footer_start] = outgoing->checksumA;
	tx[footer_start + 1] = outgoing->checksumB;

	//calculate the checksum
	calculateChecksum(outgoing, tx, tx_size);

	cs_low(config); //begin the transmission
	HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(config->spi_port, tx, rx_buf, tx_size, max_wait);
	cs_high(config); //end the transaction

	print_HAL_Status(result);

	//we may need to look for ACK depnding on type of command
	if (outgoing->cls == UBX_CLASS_CFG) {
		//call waitforack here which should return some sort of status
	}
}

void neom9n_begin(NeoGPSConfig_t *config) {
	cs_high(config);
	//todo later
}
