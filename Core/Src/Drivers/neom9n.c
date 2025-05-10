#include "Drivers/neom9n.h"
#include <string.h>

//INTERFACE DESCRIPTION:
//https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf?utm_content=UBX-21022436

ublox_status_e sendSPICommand(NeoGPSConfig_t *config, UBX_Packet_t *outgoing, uint32_t max_wait);

#define SPI_RX_BUFFER_SIZE 128

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

	HAL_Delay(1000);
	debug_print("Created tx buffer, calculating checksum:\r\n");
	//calculate the checksum
	calculateChecksum(outgoing, tx, tx_size-UBX_PACKET_FOOTER_SIZE); //subtract footer because we do not include checksums in calculation
	int footer_start = UBX_PACKET_HEADER_SIZE + outgoing->length;

	tx[footer_start] = outgoing->checksumA;
	tx[footer_start + 1] = outgoing->checksumB;


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

bool setSPIOutputModeToUBX(uint32_t max_wait) {
	//if (getPortSettings(PORT_ID_SPI, max_wait) == false)
	//    return (false); // Something went wrong. Bail.
	return false;
}

float parseLatLong(uint8_t *rx){
	//grab the degrees part
	uint8_t deg = rx[0] * 10 + rx[1];

	//grab the minutes (rx[5] is the decimal
	float minutes = 0;
	float place = 10;

	for (int i = 0; i < 10; i++) {
		minutes += place * rx[2+i];
		place/=10;
	}

	//final answer
	float lat = (float)deg + (minutes/60);

	return lat;
}

uint8_t GLOBAL_HIGH_TX[SPI_RX_BUFFER_SIZE] = {[0 ... SPI_RX_BUFFER_SIZE-1] = 0xFF};

float latitude_deg = 0;
float longitude_deg = 0;

bool receive_nmea_payload_section(SPI_HandleTypeDef *spi_port, uint8_t *tx, uint8_t *rx, uint16_t size, uint32_t max_wait) { //stop at , or size bytes
	//read first byte to see if it's a comma (stop reading)
	HAL_SPI_TransmitReceive(spi_port, tx, rx, 1, max_wait);

	if (rx[0] == ',') return false;

	//otherwise continue to read the rest into rx
	HAL_SPI_TransmitReceive(spi_port, tx, rx+1, size-1, max_wait);

	return true;
}

//,time,lat,ns,lon,ew,quality,numsv,hdop,alt
void read_nmea_gga(NeoGPSConfig_t *config, uint32_t max_wait) {
	uint8_t rx[NMEA_PAYLOAD_SIZE];

	//receive the time ( hhmmss.ss) + comma after
	receive_nmea_payload_section(config->spi_port, GLOBAL_HIGH_TX, rx, 8+1, max_wait);


	//receive the latitude(ddmm.mmmmm)+ comma after
	if (receive_nmea_payload_section(config->spi_port, GLOBAL_HIGH_TX, rx, 10+1, max_wait) == true) {
		latitude_deg = parseLatLong(rx);
	}

	//receive the NS indicator and comma after
	uint8_t ns_indicator;
	receive_nmea_payload_section(config->spi_port, GLOBAL_HIGH_TX, &ns_indicator, 1+1, max_wait);

	//receive the longitude(ddmm.mmmmm)+ comma after
	if (receive_nmea_payload_section(config->spi_port, GLOBAL_HIGH_TX, rx, 10+1, max_wait) == true) {
		longitude_deg = parseLatLong(rx);
	}

	//receive the EW indicator and comma after
	uint8_t ew_indicator;
	receive_nmea_payload_section(config->spi_port, GLOBAL_HIGH_TX, &ew_indicator, 1+1, max_wait);

	//receive the quality and comma after
	//uint8_t quality;
	//receive_nmea_payload_section(config->spi_port, GLOBAL_HIGH_TX, &quality, 1+1, max_wait);

}

//,lat,NS,lon,EW,time,status,posMode*cs\r\n
void read_nmea_gll(NeoGPSConfig_t *config, uint32_t max_wait) {
	uint8_t rx[NMEA_PAYLOAD_SIZE];

	//receive the latitude(ddmm.mmmmm)+ comma after
	receive_nmea_payload_section(config->spi_port, GLOBAL_HIGH_TX, rx, 10+1, max_wait);

	float lat = parseLatLong(rx);

	//receive the NS indicator and comma after
	uint8_t ns_indicator;
	receive_nmea_payload_section(config->spi_port, GLOBAL_HIGH_TX, &ns_indicator, 1+1, max_wait);

	//receive the longitude(ddmm.mmmmm)+ comma after
	receive_nmea_payload_section(config->spi_port, GLOBAL_HIGH_TX, rx, 10+1, max_wait);

	float lon = parseLatLong(rx);

	//receive the EW indicator and comma after
	uint8_t ew_indicator;
	receive_nmea_payload_section(config->spi_port, GLOBAL_HIGH_TX, &ew_indicator, 1+1, max_wait);

	//ignore the rest and output our data
	latitude_deg = lat;
	longitude_deg = lon;
}

//,time,lat,NS,lon,EW,posMode,numSV,HDOP,alt,sep,diffAge,diffStation,navStatus*c ↲s\r\n
void read_nmea_gns(NeoGPSConfig_t *config, uint32_t max_wait) {
	//for our purposes the gns and gga have the same data we want and in the same format
	//some of the data they carry after lat and lon is different but we don't care, so parse
	//them as the same to save me time. I am lazy.
	read_nmea_gga(config, max_wait);
}

//,time,status,lat,NS,lon,EW,spd,cog,date,mv,mvEW,posMode,navStatus*cs\r\n
void read_nmea_rmc(NeoGPSConfig_t *config, uint32_t max_wait) {
	uint8_t rx[NMEA_PAYLOAD_SIZE];

	//receive the time ( hhmmss.ss) + comma after
	HAL_SPI_TransmitReceive(config->spi_port, GLOBAL_HIGH_TX, rx, 8+1, max_wait);

	//receive the status (don't care)
	HAL_SPI_TransmitReceive(config->spi_port, GLOBAL_HIGH_TX, rx, 1+1, max_wait);

	//receive the latitude(ddmm.mmmmm)+ comma after
	HAL_SPI_TransmitReceive(config->spi_port, GLOBAL_HIGH_TX, rx, 10+1, max_wait);

	float lat = parseLatLong(rx);

	//receive the NS indicator and comma after
	uint8_t ns_indicator;
	HAL_SPI_TransmitReceive(config->spi_port, GLOBAL_HIGH_TX, &ns_indicator, 1+1, max_wait);

	//receive the longitude(ddmm.mmmmm)+ comma after
	HAL_SPI_TransmitReceive(config->spi_port, GLOBAL_HIGH_TX, rx, 10+1, max_wait);

	float lon = parseLatLong(rx);

	//receive the EW indicator and comma after
	uint8_t ew_indicator;
	HAL_SPI_TransmitReceive(config->spi_port, GLOBAL_HIGH_TX, &ew_indicator, 1+1, max_wait);

	//receive the quality and comma after
	uint8_t quality;
	HAL_SPI_TransmitReceive(config->spi_port, GLOBAL_HIGH_TX, &quality, 1+1, max_wait);

	//ignore the rest and output our data
	latitude_deg = lat;
	longitude_deg = lon;
}


bool receive_nmea(NeoGPSConfig_t *config, uint32_t max_wait, uint32_t max_ignores) {
	HAL_Delay(10);
	debug_print("Receiving a nmea...");
	HAL_Delay(10);

	uint8_t rx[5];

	//begin the transaction
	cs_low(config);

	uint32_t ignores = 0;
	//ignore until we find the start of a nmea ($)
	while(ignores < max_ignores) {
		HAL_SPI_TransmitReceive(config->spi_port, GLOBAL_HIGH_TX, rx, 1, max_wait);
		if ((char)rx[0] == '$')
			break;
		ignores++;
	}

	if ((char)rx[0]!='$') {
		cs_high(config);
		return false; //we did not get a message after max ignores
	}
	//otherwise continue parsing the message

	uint32_t message_index = 0;
	//receive the talker id
	HAL_SPI_TransmitReceive(config->spi_port, GLOBAL_HIGH_TX, rx, 2, max_wait);
	message_index+=2;

	//receive the sentence format
	//add 1 to the length of format to account for the comma
	HAL_SPI_TransmitReceive(config->spi_port, GLOBAL_HIGH_TX, rx+message_index, 3+1, max_wait);

	//now we are up to the payload, so we should pass control to the relevant parsing function based on sentence type.
	//first check if it is a ublox proprietary message (PUBX)

	char *sent = (char*)rx + 2;
	if (strncmp((char*)rx, "PUBX", 4) == 0) {
		//I have no need for this data at this time, so return false and ignore.
		cs_high(config);
		debug_print("PUBX: Ignore"); //should never happen, because we only get this if we poll for it.
		return false;
	} else { //else check the sentence format field
		if (strncmp(sent, "GGA", 3) == 0) {
			read_nmea_gga(config, max_wait);
			debug_print("GGA parsed. Latitude: %f, Longitude: %f", latitude_deg, longitude_deg);
		} else if (strncmp(sent, "GLL", 3) == 0) {
			read_nmea_gll(config, max_wait);
			debug_print("GLL parsed. Latitude: %f, Longitude: %f", latitude_deg, longitude_deg);
		} else if (strncmp(sent, "GNS", 3) == 0) {
			read_nmea_gns(config, max_wait);
			debug_print("GNS parsed. Latitude: %f, Longitude: %f", latitude_deg, longitude_deg);
		} else if (strncmp(sent, "RMC", 3) == 0) {
			read_nmea_rmc(config, max_wait);
			debug_print("RMC parsed. Latitude: %f, Longitude: %f", latitude_deg, longitude_deg);
		} else {
			cs_high(config);
			debug_print("%c%c%c: Ignore", rx[2], rx[3], rx[4]);
			return false;
		}
	}
	return true;
}

bool neom9n_begin(NeoGPSConfig_t *config, uint32_t max_wait) {
	debug_print("Beginning...\r\n");
	HAL_Delay(2000);
	cs_high(config);
	//test_poll(config, max_wait);
	while(1) {
		receive_nmea(config,max_wait, 1000);
		HAL_Delay(100);
	}
	//we should ideally attempt to connect a few times
	return true;
}
