#include "Drivers/neom9n.h"

//INTERFACE DESCRIPTION:
//https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf?utm_content=UBX-21022436

ublox_status_e sendSPICommand(NeoGPSConfig_t *config, UBX_Packet_t *outgoing, uint32_t max_wait);

#define SPI_RX_BUFFER_SIZE 128*4//128

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
 * Process a byte from the spi buffer and return true if a valid byte was processsed
 * based on the given frame_count
 *
 * FRAME FORMAT FOR UBX:
 * HEADER | Class | ID | Length | Payload | CK_A | CK_B |
 * 0    1   2       3    4    5   6         6+n    7+n
 */
bool processSpiByte(uint8_t incoming, UBX_Packet_t *incoming_packet) {
	uint16_t frame_count = incoming_packet->recv_counter;
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
		incoming_packet->class = incoming;
		debug_print("CLS %02x | ", incoming);
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
void checkAndProcessSPIBuffer(UBX_Packet_t *outgoing, uint8_t *rx_buf, uint16_t rx_size, UBX_Packet_t *incoming_packet) {
	//overhead for processspibyte
	for(int i = 0; i < rx_size; i++) {
		debug_print("(%02x)", rx_buf[i]);
		if (processSpiByte(rx_buf[i], incoming_packet) == true) {
			incoming_packet->recv_counter ++;;
		} else {
			debug_print("processing spi byte failed.\n");
		}
		HAL_Delay(100);
	}
}

//ACK MESSAGE IS IN THE FORMAT HEADER | CLASS (0X05) | ID (0X01 FOR ACK 0X00 FOR NOT ACK) | len | payload | checksum
bool waitForAck(UBX_Packet_t  *outgoing, uint8_t *rx_buf, uint16_t rx_size, uint32_t max_wait) {
	//we don't actually need to wait like in the arduino driver, because the stm32 hal functions are all blocking
	//with a given timeout.
	UBX_Packet_t incoming_packet;
	incoming_packet.recv_counter = 0;
	HAL_Delay(1000);
	debug_print("doing wait for ack...\r\n");
	HAL_Delay(1000);
	checkAndProcessSPIBuffer(outgoing, rx_buf, rx_size, &incoming_packet); // See if new data is available. Process bytes as they come in.

	//now we should check if the stuff in incoming_packet is valid.

	if (incoming_packet.class == outgoing->class) {
		debug_print("Classes match!\n");
	} else {
		debug_print("Classes don't match.\n");
		return false;
	}

	if (incoming_packet.id == outgoing->id) {
		debug_print("Ids match!\n");
	} else {
		debug_print("Ids don't match.\n");
		return false;
	}

	return true;
}

ublox_status_e sendSPICommand(NeoGPSConfig_t *config, UBX_Packet_t *outgoing, uint32_t max_wait) {
	//start with header bytes
	//determine size of
	debug_print("Running sendspicommand...\r\n");
	uint16_t tx_size = UBX_PACKET_HEADER_SIZE +
			   outgoing->length +
			   UBX_PACKET_FOOTER_SIZE;

	uint8_t tx[tx_size*2];

	tx[0] = UBX_PSYNC_1;
	tx[1] = UBX_PSYNC_2;

	tx[2] = outgoing->class;
	tx[3] = outgoing->id;
	tx[4] = outgoing->length & 0xFF; //LSB OF LENGTH,
	tx[5] = outgoing->length >> 8;   //MSB OF LENGTH. LENGTH IS LSB FIRST PER DATASHEET

	HAL_Delay(1000);
	debug_print("Set headers, setting payload of length %d...\r\n", outgoing->length);
	HAL_Delay(1000);
	for (int i = 0; i < outgoing->length; i++) {
		tx[UBX_PACKET_HEADER_SIZE + i] = outgoing->payload[i];
	}
	HAL_Delay(1000);
	debug_print("Set payload.\r\n");
	int footer_start = UBX_PACKET_HEADER_SIZE + outgoing->length;

	tx[footer_start] = outgoing->checksumA;
	tx[footer_start + 1] = outgoing->checksumB;

	HAL_Delay(1000);
	debug_print("Created tx buffer, calculating checksum:\r\n");
	//calculate the checksum
	calculateChecksum(outgoing, tx, tx_size);
	HAL_Delay(1000);
	debug_print("Calculated checksum, filling end of tx and beginning of rx...\r\n");
	HAL_Delay(500);

	uint8_t rx_buf[tx_size*2];

	for(int i = 0; i < tx_size; i++) {
		tx[tx_size+i] = 0xFF;
		//rx_buf[i] = 0xFF;
	}

	HAL_Delay(500);
	debug_print("Done filling. Begin transaction...\r\n");

	cs_low(config); //begin the transmission
	HAL_StatusTypeDef res = HAL_SPI_TransmitReceive(config->spi_port, tx, rx_buf, tx_size*2, max_wait);
	cs_high(config); //end the transaction
	HAL_Delay(1000);
	debug_print("Spi transaction over (%02x). Parsing results...\r\n", res);
	HAL_Delay(1000);
	debug_print("Debug printing rx and tx buffers:\r\n");
	HAL_Delay(500);
	debug_print("TX: ");
	for(int i = 0; i < tx_size*2; i++) {
		HAL_Delay(100);
		debug_print(" |%02x| ", tx[i]);
	}
	HAL_Delay(500);
	debug_print("\r\nRX: ");
	for(int i = 0; i < tx_size*2; i++) {
		HAL_Delay(100);
		debug_print(" |%02x| ", rx_buf[i]);
	}
	//print_HAL_Status(result);

	//sfe_ublox_status_e ret = UBLOX_STATUS_SUCCESS;
	//bool ret = false;
	//we may need to look for ACK depnding on type of command
	if (outgoing->class == UBX_CLASS_CFG) {
		//call waitforack here which should return some sort of status
		waitForAck(outgoing, rx_buf, tx_size*2, max_wait);
	} else {
		//waitForNoAck(outgoing, max_wait);
	}

	return UBLOX_STATUS_DATA_RECEIVED;
}

bool getSPIPortSettings(NeoGPSConfig_t *config, uint32_t max_wait)
{
	UBX_Packet_t packet;
	packet.class = UBX_CLASS_CFG;
	packet.id = UBX_CFG_PRT;
	packet.length = 1;

	uint8_t payload = PORT_ID_SPI;

	packet.payload = &payload;

	debug_print("Set up port settings poll packet, sending command:\r\n");
	HAL_Delay(1000);
	return ((sendSPICommand(config, &packet, max_wait)) == UBLOX_STATUS_DATA_RECEIVED); // We are expecting data and an ACK
}

/*
 * To check if the device is connected,
 * we check the SPI port settings of some
 * port to see if we get a valid result.
 */
bool isConnected(NeoGPSConfig_t *config, uint32_t max_wait) {
	debug_print("Checking connection...\r\n");
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

bool setSPIOutputModeToUBX(uint32_t max_wait) {
	//if (getPortSettings(PORT_ID_SPI, max_wait) == false)
	//    return (false); // Something went wrong. Bail.
	return false;
}

void test_poll(NeoGPSConfig_t *config, uint32_t max_wait) {
	HAL_Delay(500);
	debug_print("Starting test polling...");
	HAL_Delay(500);

	uint8_t tx[SPI_RX_BUFFER_SIZE] = {[0 ... SPI_RX_BUFFER_SIZE-1] = 0xFF};

	tx[5] = UBX_PSYNC_1;
	tx[6] = UBX_PSYNC_2;

	tx[7] = UBX_CLASS_CFG;
	tx[8] = UBX_CFG_PRT;
	tx[9] = 1 & 0xFF; //LSB OF LENGTH,
	tx[10] = 1 >> 8;   //MSB OF LENGTH. LENGTH IS LSB FIRST PER DATASHEET



	uint8_t rx[SPI_RX_BUFFER_SIZE];

	while(1) {
		debug_print("POLLING...");
		cs_low(config);
		HAL_SPI_TransmitReceive(config->spi_port, tx, rx, SPI_RX_BUFFER_SIZE, max_wait);
		cs_high(config);
		HAL_Delay(100);
		debug_print("Transaction done.\r\n");
		HAL_Delay(100);
		debug_print("Debug printing rx and tx buffers:\r\n");
		HAL_Delay(100);
		debug_print("TX: ");
		for(int i = 0; i < SPI_RX_BUFFER_SIZE; i++) {
			HAL_Delay(10);
			debug_print(" |%02x| ", tx[i]);
		}
		HAL_Delay(100);
		debug_print("\r\nRX: ");
		for(int i = 0; i < SPI_RX_BUFFER_SIZE; i++) {
			HAL_Delay(10);
			debug_print(" |%02x| ", rx[i]);
		}
		HAL_Delay(100);
		debug_print("\r\nDone. Waiting 5 seconds...\r\n");
		HAL_Delay(5000);
	}
}

bool neom9n_begin(NeoGPSConfig_t *config, uint32_t max_wait) {
	debug_print("Beginning...\r\n");
	HAL_Delay(2000);
	cs_high(config);

	bool connected = isConnected(config, max_wait);
	test_poll(config, max_wait);
	//we should ideally attempt to connect a few times
	return connected;
}
