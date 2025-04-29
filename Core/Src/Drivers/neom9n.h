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

//class bytes
const uint8_t UBX_CLASS_ACK =   0x05;
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

//id bytes for config class
// Class: CFG
// The following are used for configuration. Descriptions are from the ZED-F9P Interface Description pg 33-34 and NEO-M9N Interface Description pg 47-48
const uint8_t UBX_CFG_ANT = 0x13;       // Antenna Control Settings. Used to configure the antenna control settings
const uint8_t UBX_CFG_BATCH = 0x93;     // Get/set data batching configuration.
const uint8_t UBX_CFG_CFG = 0x09;       // Clear, Save, and Load Configurations. Used to save current configuration
const uint8_t UBX_CFG_DAT = 0x06;       // Set User-defined Datum or The currently defined Datum
const uint8_t UBX_CFG_DGNSS = 0x70;     // DGNSS configuration
const uint8_t UBX_CFG_ESFALG = 0x56;    // ESF alignment
const uint8_t UBX_CFG_ESFA = 0x4C;      // ESF accelerometer
const uint8_t UBX_CFG_ESFG = 0x4D;      // ESF gyro
const uint8_t UBX_CFG_GEOFENCE = 0x69;  // Geofencing configuration. Used to configure a geofence
const uint8_t UBX_CFG_GNSS = 0x3E;      // GNSS system configuration
const uint8_t UBX_CFG_HNR = 0x5C;       // High Navigation Rate
const uint8_t UBX_CFG_INF = 0x02;       // Depending on packet length, either: poll configuration for one protocol, or information message configuration
const uint8_t UBX_CFG_ITFM = 0x39;      // Jamming/Interference Monitor configuration
const uint8_t UBX_CFG_LOGFILTER = 0x47; // Data Logger Configuration
const uint8_t UBX_CFG_MSG = 0x01;       // Poll a message configuration, or Set Message Rate(s), or Set Message Rate
const uint8_t UBX_CFG_NAV5 = 0x24;      // Navigation Engine Settings. Used to configure the navigation engine including the dynamic model.
const uint8_t UBX_CFG_NAVX5 = 0x23;     // Navigation Engine Expert Settings
const uint8_t UBX_CFG_NMEA = 0x17;      // Extended NMEA protocol configuration V1
const uint8_t UBX_CFG_ODO = 0x1E;       // Odometer, Low-speed COG Engine Settings
const uint8_t UBX_CFG_PM2 = 0x3B;       // Extended power management configuration
const uint8_t UBX_CFG_PMS = 0x86;       // Power mode setup
const uint8_t UBX_CFG_PRT = 0x00;       // Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
const uint8_t UBX_CFG_PWR = 0x57;       // Put receiver in a defined power state
const uint8_t UBX_CFG_RATE = 0x08;      // Navigation/Measurement Rate Settings. Used to set port baud rates.
const uint8_t UBX_CFG_RINV = 0x34;      // Contents of Remote Inventory
const uint8_t UBX_CFG_RST = 0x04;       // Reset Receiver / Clear Backup Data Structures. Used to reset device.
const uint8_t UBX_CFG_RXM = 0x11;       // RXM configuration
const uint8_t UBX_CFG_SBAS = 0x16;      // SBAS configuration
const uint8_t UBX_CFG_TMODE3 = 0x71;    // Time Mode Settings 3. Used to enable Survey In Mode
const uint8_t UBX_CFG_TP5 = 0x31;       // Time Pulse Parameters
const uint8_t UBX_CFG_USB = 0x1B;       // USB Configuration
const uint8_t UBX_CFG_VALDEL = 0x8C;    // Used for config of higher version u-blox modules (ie protocol v27 and above). Deletes values corresponding to provided keys/ provided keys with a transaction
const uint8_t UBX_CFG_VALGET = 0x8B;    // Used for config of higher version u-blox modules (ie protocol v27 and above). Configuration Items
const uint8_t UBX_CFG_VALSET = 0x8A;    // Used for config of higher version u-blox modules (ie protocol v27 and above). Sets values corresponding to provided key-value pairs/ provided key-value pairs within a transaction.

const uint8_t PORT_ID_SPI = 4; //spi com port number for polling configs

const uint16_t UBX_PACKET_HEADER_SIZE = 6; //1(8) + 1(8) + 1(8) + 1(8) + 2(16)
const uint16_t UBX_PACKET_FOOTER_SIZE = 2; //2 checksum bytes

typedef struct NeoGPSConfig_t {
	SPI_HandleTypeDef spi_port;
	GPIO_TypeDef *cs_pin_port;
	uint16_t cs_pin;
} NeoGPSConfig_t;

typedef struct UBX_Packet_t {
	uint8_t class;
	uint8_t id;
	uint8_t length;
	uint8_t *payload;
	uint16_t payload_length;
	uint8_t checksumA;
	uint8_t checksumB;
	//uint8_t
} UBX_Packet_t;

typedef enum incoming_packet_type_e {
	INCOMING_TYPE_NONE = 0,
	INCOMING_TYPE_NMEA,
	INCOMING_TYPE_UBX,
	INCOMING_TYPE_RTCM
} incoming_packet_type_e;

typedef enum ublox_status_e
{
  UBLOX_STATUS_SUCCESS,
  UBLOX_STATUS_FAIL,
  UBLOX_STATUS_CRC_FAIL,
  UBLOX_STATUS_TIMEOUT,
  UBLOX_STATUS_COMMAND_NACK, // Indicates that the command was unrecognized, invalid or that the module is too busy to respond
  UBLOX_STATUS_OUT_OF_RANGE,
  UBLOX_STATUS_INVALID_ARG,
  UBLOX_STATUS_INVALID_OPERATION,
  UBLOX_STATUS_MEM_ERR,
  UBLOX_STATUS_HW_ERR,
  UBLOX_STATUS_DATA_SENT,     // This indicates that a 'set' was successful
  UBLOX_STATUS_DATA_RECEIVED, // This indicates that a 'get' (poll) was successful
  UBLOX_STATUS_I2C_COMM_FAILURE,
  UBLOX_STATUS_DATA_OVERWRITTEN // This is an error - the data was valid but has been or _is being_ overwritten by another packet
} ublox_status_e;

#endif /* SRC_DRIVERS_NEOM9N_H_ */
