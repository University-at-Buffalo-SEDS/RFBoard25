#include "main.h"

//INTERFACE DESCRIPTION:
//https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf?utm_content=UBX-21022436

//sync characters
const uint8_t UBX_PSYNC_1 = 0xB5; //Preamble sync char 1
const uint8_t UBX_PSYNC_2 = 0x62; //Preamble sync char 2

const uint16_t UBX_PACKET_HEADER_SIZE = 6; //1(8) + 1(8) + 1(8) + 1(8) + 2(16)
const uint16_t UBX_PACKET_FOOTER_SIZE = 2; //2 checksum bytes

typedef struct NeoGPSConfig_t {
	SPI_HandleTypeDef spi_port;
	GPIO_TypeDef *cs_pin_port;
	uint16_t cs_pin;
} NeoGPSConfig_t;

typedef struct {
	uint8_t class;
	uint8_t id;
	uint8_t length;
	uint8_t *payload;
	uint16_t payload_length;
	uint8_t checksumA;
	uint8_t checksumB;
	//uint8_t
} UBX_Packet_t;

static void cs_low(NeoGPSConfig_t *config) {
	HAL_GPIO_WritePin(config->cs_pin_port, config->cs_pin, GPIO_PIN_RESET);
}

static void cs_high(NeoGPSConfig_t *config) {
	HAL_GPIO_WritePin(config->cs_pin_port, config->cs_pin, GPIO_PIN_SET);
}

void sendSPICommand(NeoGPSConfig_t *config, UBX_Packet_t *outgoing) {
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

	cs_low(config); //begin the transmission
	HAL_SPI_Transmit(config->spi_port, tx, tx_size);
	cs_high(config); //end the transaction
}

void neom9n_begin(NeoGPSConfig_t *config) {
	cs_high(config);
	//todo later
}
