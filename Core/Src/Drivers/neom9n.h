/*
 * neom9n.h
 *
 *  Created on: Apr 25, 2025
 *      Author: fredd
 */

#ifndef SRC_DRIVERS_NEOM9N_H_
#define SRC_DRIVERS_NEOM9N_H_


//INTERFACE DESCRIPTION:
//https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf?utm_content=UBX-21022436

//sync characters
const uint8_t UBX_PSYNC_1 = 	0xB5; //Preamble sync char 1
const uint8_t UBX_PSYNC_2 = 	0x62; //Preamble sync char 2

const uint8_t UBX_CLASS_CFG = 	0x06; //Class for configuration packets

const uint8_t UBX_CFG_PRT = 	0x00;


const uint8_t PORT_ID_SPI = 4; //spi com port number for polling configs

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

#endif /* SRC_DRIVERS_NEOM9N_H_ */
