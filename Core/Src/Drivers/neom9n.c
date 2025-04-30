#include "main.h"
#include "neom9n.h"
#include <stdbool.h>

//INTERFACE DESCRIPTION:
//https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf?utm_content=UBX-21022436

ublox_status_e sendSPICommand(NeoGPSConfig_t *config, UBX_Packet_t *outgoing, uint16_t max_wait);

ublox_status_e waitForAck(UBX_Packet_t  *outgoing, uint16_t max_wait);
ublox_status_e waitForNoAck(UBX_Packet_t  *outgoing, uint16_t max_wait);

#define SPI_RX_BUFFER_SIZE 128

uint8_t spi_rx_buffer[SPI_RX_BUFFER_SIZE];
uint8_t spi_buffer_index = 0; //currently occupied bytes of spi rx buffer

//todo make this actually print something
static void print_HAL_Status(HAL_StatusTypeDef status) {
	switch (status) {
	case HAL_OK:
		debug_print("HAL_OK");
		break;
	case HAL_ERROR:
		debug_print("HAL_ERROR");
		break;
	case HAL_BUSY:
		debug_print("HAL_BUSY");
		break;
	case HAL_TIMEOUT:
		debug_print("HAL_TIMEOUT");
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

/*
 * const uint8_t UBX_CLASS_ACK =   0x05;
const uint8_t UBX_CLASS_CFG = 	0x06; //Class for configuration packets
const uint8_t UBX_CLASS_INF =   0x04;
const uint8_t UBX_CLASS_LOG =   0x21;
const uint8_t UBX_CLASS_MGA =   0x13;
const uint8_t UBX_CLASS_MON =   0x0a;
const uint8_t UBX_CLASS_NAV =   0x01;
const uint8_t UBX_CLASS_RXM =   0x02;
const uint8_t UBX_CLASS_SEC =   0x27;
const uint8_t UBX_CLASS_TIM =   0x0d;
const uint8_t UBX_CLASS_UPD =   0x09;
 */


static bool isValidClass(uint8_t class_byte) {
	if (
	class_byte == UBX_CLASS_ACK ||
	class_byte == UBX_CLASS_CFG ||
	class_byte == UBX_CLASS_INF ||
	class_byte == UBX_CLASS_LOG ||
	class_byte == UBX_CLASS_MGA ||
	class_byte == UBX_CLASS_MON ||
	class_byte == UBX_CLASS_NAV ||
	class_byte == UBX_CLASS_RXM ||
	class_byte == UBX_CLASS_SEC ||
	class_byte == UBX_CLASS_TIM ||
	class_byte == UBX_CLASS_UPD0
	) {
		return true;
	} else {
		return false;
	}
}

static bool isValidCFGByte(uint8_t cfg_byte) {
	for (int i = 0; i < sizeof(LOOKUP_UBX_CFG); i++) {
		if (cfg_byte == LOOKUP_UBX_CFG[i]) {
			return true;
		}
	}
	return false;
}

/*
 * Process a byte from the spi buffer and return true if a valid byte was processsed
 * based on the given frame_count
 *
 * FRAME FORMAT FOR UBX:
 * HEADER | Class | ID | Length | Payload | CK_A | CK_B |
 * 0    1   2       3    4    5   6         6+n    7+n
 */
bool processSpiByte(uint8_t incoming, UBX_Packet_t *incoming_packet, uint16_t frame_count) {
	if (frame_count == 0) {
		if (incoming == UBX_PSYNC_1) { //psync_1 (b5) is the beginning of a ubx message
			debug_print("PSYNC_1 | ");
		} else if (incoming == '$') { //$ is the beginning of a nmea message (see interface document page 21)
			//cur_incoming_type = INCOMING_TYPE_NMEA;
			debug_print("== WARNING: DRIVER DOES NOT SUPPORT NMEA ==\r\n");
			return false;
		} else if (incoming == 0xD3) { //0xd3 is the beginning of a rtcm message
			debug_print("== WARNING: DRIVER DOES NOT SUPPORT RTCM ==\r\n");
			return false;
		}
	} else if (frame_count == 1) {
		if (incoming == UBX_PSYNC_2) {
			//psync_2 is the second header byte of a ubx message
			debug_print("PSYNC_2 | ");
		} else {
			return false;
		}
	} else if (frame_count == 2) { //should be the class, store it in the given struct
		if (isValidClass(inncoming) == true) {
			incoming_packet->class = incoming;
			debug_print("CLS %02x | ", incoming);
		} else {
			return false;
		}
	} else if (frame_count == 3) { //should be the id
		debug_print("ID %02x | ", incoming);
		incoming_packet->id = incoming;
	} else if (frame_count == 4) { //length LSB
		//do this in reverse
		incoming_packet->length = incoming;
	} else if (frame_count == 5) { //length msb
		incoming_packet->length |= incoming << 8;
	} else if (frame_count > 5) { //this is either the payload of the checksum
		if (frame_count < 6+incoming_packet->length) { //this is part of the payload
			incoming_packet->payload[frame_count-6] = incoming;
		} else { //these are the checksum
			if (frame_count == 6 + incoming_packet->length) {
				incoming_packet->checksumA = incoming;
			} else if (frame_count == 7 + incoming_packet->length){
				incoming_packet->checksumB = incoming;
			} else {
				return false;
			}
		}
	} else {
		return false; //invalid frame_count (not sure why this would happen)
	}
	return true;
}

/*
 * Go through the SPI buffer and process incoming bytes
 * into the given incoming_packet struct, comparing the
 * incoming to the outgoing (id and class of incoming
 * should match the outgoing)
 */
bool checkAndProcessSPIBuffer(UBX_Packet_t *outgoing, UBX_Packet_t *incoming_packet) {
	//overhead for processspibyte
	incoming_packet_type_e cur_incoming_type = INCOMING_TYPE_NONE;
	uint16_t cur_incoming_frame_count = 0;

	for(int i = 0; i < spi_buffer_index; i++) {
		if (processSpiByte(spi_rx_buffer[i], incoming_packet, cur_incoming_frame_count) == true) {
			cuur_incoming_frame_count ++;;
		} else {
			return false; //one of the bytes we are reading is invalid
			//processspibyte will print a debug message if necessary
		}
	}
	spi_buffer_index = 0;
}

//ACK MESSAGE IS IN THE FORMAT HEADER | CLASS (0X05) | ID (0X01 FOR ACK 0X00 FOR NOT ACK) | len | payload | checksum
ublox_status_e waitForAck(UBX_Packet_t  *outgoing, uint16_t max_wait) {
	//we don't actually need to wait like in the arduino driver, because the stm32 hal functions are all blocking
	//with a given timeout.
	if (checkAndProcessSPIBuffer(outgoing) == true) { // See if new data is available. Process bytes as they come in.
		return (UBLOX_STATUS_DATA_RECEIVED); // We received valid data and a correct ACK!
	} else {
		return UBLOX_STATUS_TIMEOUT;
	}
	//}
}
/*
sfe_ublox_status_e SFE_UBLOX_GNSS::waitForACKResponse(ubxPacket *outgoingUBX, uint8_t requestedClass, uint8_t requestedID, uint16_t maxTime)
{
	if (checkUbloxInternal(outgoingUBX, requestedClass, requestedID) == true) // See if new data is available. Process bytes as they come in.
	{
	  if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
	  {
		return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and a correct ACK!
	  }
	  else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID))
	  {
		return (SFE_UBLOX_STATUS_DATA_SENT); // We got an ACK but no data...
	  }
	  else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && ((outgoingUBX->cls != requestedClass) || (outgoingUBX->id != requestedID)))
	  {
		return (SFE_UBLOX_STATUS_DATA_OVERWRITTEN); // Data was valid but has been or is being overwritten
	  }
	  else if ((packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
	  {
		return (SFE_UBLOX_STATUS_CRC_FAIL); // Checksum fail
	  }
	  else if (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_NOTACKNOWLEDGED)
	  {
		return (SFE_UBLOX_STATUS_COMMAND_NACK); // We received a NACK!
	  }
	  else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
	  {
		return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data and an invalid ACK!
	  }
	  else if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_VALID))
	  {
		return (SFE_UBLOX_STATUS_FAIL); // We received invalid data and an invalid ACK!
	  }

	} // checkUbloxInternal == true

	delay(1); // Allow an RTOS to get an elbow in (#11)

	// We have timed out...
	// If the outgoingUBX->classAndIDmatch is VALID then we can take a gamble and return DATA_RECEIVED
	// even though we did not get an ACK
	if ((outgoingUBX->classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_VALID) && (packetAck.classAndIDmatch == SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED) && (outgoingUBX->valid == SFE_UBLOX_PACKET_VALIDITY_VALID) && (outgoingUBX->cls == requestedClass) && (outgoingUBX->id == requestedID))
	{
	return (SFE_UBLOX_STATUS_DATA_RECEIVED); // We received valid data... But no ACK!
	}
	return (SFE_UBLOX_STATUS_TIMEOUT);
}*/


ublox_status_e sendSPICommand(NeoGPSConfig_t *config, UBX_Packet_t *outgoing, uint16_t max_wait) {
	spi_buffer_index = 0; //start at beginning of buffer
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

	uint8_t rx_buf[SPI_RX_BUFFER_SIZE];

	cs_low(config); //begin the transmission
	HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(config->spi_port, tx, rx_buf, tx_size, max_wait);
	cs_high(config); //end the transaction

	//should put the rx data into the global spi rx buffer
	for (int i = 0; i < tx_size; i++) {
		spi_rx_buffer[spi_buffer_index] = rx_buf[i];
		spi_buffer_index++;
	}

	print_HAL_Status(result);

	sfe_ublox_status_e ret = UBLOX_STATUS_SUCCESS;

	//we may need to look for ACK depnding on type of command
	if (outgoing->class == UBX_CLASS_CFG) {
		//call waitforack here which should return some sort of status
		ret = waitForAck(outgoing, max_wait);
	} else {
		ret = waitForNoAck(outgoing, max_wait);
	}
}

bool getSPIPortSettings(NeoGPSConfig_t *config, uint16_t max_wait)
{
	UBX_Packet_t packet;
	packet.class = UBX_CLASS_CFG;
	packet.id = UBX_CFG_PRT;
	packet.length = 1;

	uint8_t payload = PORT_ID_SPI;

	packet.payload = &payload;

	return ((sendSPICommand(config, &packet, max_wait)) == UBLOX_STATUS_DATA_RECEIVED); // We are expecting data and an ACK
}

/*
 * To check if the device is connected,
 * we check the SPI port settings of some
 * port to see if we get a valid result.
 */
bool isConnected(NeoGPSConfig_t *config, uint16_t max_wait) {
	return getSPIPortSettings(config, max_wait);
}


/*
 * / Configure a given port to output UBX, NMEA, RTCM3 or a combination thereof
// Port 0=I2c, 1=UART1, 2=UART2, 3=USB, 4=SPI
// Bit:0 = UBX, :1=NMEA, :5=RTCM3
bool SFE_UBLOX_GNSS::setPortOutput(uint8_t portID, uint8_t outStreamSettings, uint16_t maxWait)
{
  // Get the current config values for this port ID
  if (getPortSettings(portID, maxWait) == false)
    return (false); // Something went wrong. Bail.

  packetCfg.cls = UBX_CLASS_CFG;
  packetCfg.id = UBX_CFG_PRT;
  packetCfg.len = 20;
  packetCfg.startingSpot = 0;

  // payloadCfg is now loaded with current bytes. Change only the ones we need to
  payloadCfg[14] = outStreamSettings; // OutProtocolMask LSB - Set outStream bits

  return ((sendCommand(&packetCfg, maxWait)) == SFE_UBLOX_STATUS_DATA_SENT); // We are only expecting an ACK
}
 */

bool setSPIOutputModeToUBX(uint16_t max_wait) {
	if (getPortSettings(portID, maxWait) == false)
	    return (false); // Something went wrong. Bail.
}

bool neom9n_begin(NeoGPSConfig_t *config, uint16_t max_wait) {
	cs_high(config);

	bool connected = isConnected(config, max_wait);
	//we should ideally attempt to connect a few times
	return connected;
}
