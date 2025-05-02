/*
 * neom9n.h
 *
 *  Created on: Apr 25, 2025
 *      Author: fredd
 */

#ifndef SRC_DRIVERS_NEOM9N_H_
#define SRC_DRIVERS_NEOM9N_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32g4xx_hal.h"

//INTERFACE DESCRIPTION:
//https://content.u-blox.com/sites/default/files/u-blox-M9-SPG-4.04_InterfaceDescription_UBX-21022436.pdf?utm_content=UBX-21022436

//sync characters
#define UBX_PSYNC_1  	0xB5 //Preamble sync char 1
#define UBX_PSYNC_2  	0x62 //Preamble sync char 2

//class bytes
#define UBX_CLASS_ACK    0x05
#define UBX_CLASS_CFG	 0x06 //Class for configuration packets
#define UBX_CLASS_INF    0x04
#define UBX_CLASS_LOG    0x21
#define UBX_CLASS_MGA    0x13
#define UBX_CLASS_MON    0x0a
#define UBX_CLASS_NAV    0x01
#define UBX_CLASS_RXM    0x02
#define UBX_CLASS_SEC    0x27
#define UBX_CLASS_TIM    0x0d
#define UBX_CLASS_UPD    0x09
//id bytes for config class
// Class: CFG
// The following are used for configuration. Descriptions are from the ZED-F9P Interface Description pg 33-34 and NEO-M9N Interface Description pg 47-48
#define UBX_CFG_ANT  		0x13       // Antenna Control Settings. Used to configure the antenna control settings
#define UBX_CFG_BATCH  		0x93     // Get/set data batching configuration.
#define UBX_CFG_CFG  		0x09       // Clear, Save, and Load Configurations. Used to save current configuration
#define UBX_CFG_DAT  		0x06       // Set User-defined Datum or The currently defined Datum
#define UBX_CFG_DGNSS  		0x70     // DGNSS configuration
#define UBX_CFG_ESFALG  	0x56    // ESF alignment
#define UBX_CFG_ESFA 		0x4C      // ESF accelerometer
#define UBX_CFG_ESFG  		0x4D      // ESF gyro
#define UBX_CFG_GEOFENCE  	0x69  // Geofencing configuration. Used to configure a geofence
#define UBX_CFG_GNSS  		0x3E      // GNSS system configuration
#define UBX_CFG_HNR  		0x5C       // High Navigation Rate
#define UBX_CFG_INF  		0x02       // Depending on packet length, either: poll configuration for one protocol, or information message configuration
#define UBX_CFG_ITEM  		0x39      // Jamming/Interference Monitor configuration
#define UBX_CFG_LOGFILTER  	0x47 // Data Logger Configuration
#define UBX_CFG_MSG  		0x01       // Poll a message configuration, or Set Message Rate(s), or Set Message Rate
#define UBX_CFG_NAV5  		0x24      // Navigation Engine Settings. Used to configure the navigation engine including the dynamic model.
#define UBX_CFG_NAVX5  		0x23     // Navigation Engine Expert Settings
#define UBX_CFG_NMEA  		0x17      // Extended NMEA protocol configuration V1
#define UBX_CFG_ODO  		0x1E       // Odometer, Low-speed COG Engine Settings
#define UBX_CFG_PM2  		0x3B       // Extended power management configuration
#define UBX_CFG_PMS  		0x86       // Power mode setup
#define UBX_CFG_PRT 		0x00       // Used to configure port specifics. Polls the configuration for one I/O Port, or Port configuration for UART ports, or Port configuration for USB port, or Port configuration for SPI port, or Port configuration for DDC port
#define UBX_CFG_PWR  		0x57       // Put receiver in a defined power state
#define UBX_CFG_RATE  		0x08      // Navigation/Measurement Rate Settings. Used to set port baud rates.
#define UBX_CFG_RINV 		0x34      // Contents of Remote Inventory
#define UBX_CFG_RST  		0x04       // Reset Receiver / Clear Backup Data Structures. Used to reset device.
#define UBX_CFG_RXM  		0x11       // RXM configuration
#define UBX_CFG_SBAS  		0x16      // SBAS configuration
#define UBX_CFG_TMODE3  	0x71    // Time Mode Settings 3. Used to enable Survey In Mode
#define UBX_CFG_TP5  		0x31       // Time Pulse Parameters
#define UBX_CFG_USB  		0x1B       // USB Configuration
#define UBX_CFG_VALDEL  	0x8C    // Used for config of higher version u-blox modules (ie protocol v27 and above). Deletes values corresponding to provided keys/ provided keys with a transaction
#define UBX_CFG_VALGET  	0x8B    // Used for config of higher version u-blox modules (ie protocol v27 and above). Configuration Items
#define UBX_CFG_VALSET  	0x8A    // Used for config of higher version u-blox modules (ie protocol v27 and above). Sets values corresponding to provided key-value pairs/ provided key-value pairs within a transaction.

//const uint8_t LOOKUP_UBX_CFG[] = {
//		UBX_CFG_ANT, UBX_CFG_BATCH, UBX_CFG_CFG,
//		UBX_CFG_DAT, UBX_CFG_DGNSS, UBX_CFG_ESFALG,
//		UBX_CFG_ESFA, UBX_CFG_ESFG, UBX_CFG_GEOFENCE,
//		UBX_CFG_GNSS, UBX_CFG_HNR, UBX_CFG_INF,
//		UBX_CFG_ITEM, UBX_CFG_LOGFILTER, UBX_CFG_MSG,
//		UBX_CFG_NAV5, UBX_CFG_NAVX5, UBX_CFG_NMEA,
//		UBX_CFG_ODO, UBX_CFG_PM2, UBX_CFG_PMS,
//		UBX_CFG_PRT, UBX_CFG_PWR, UBX_CFG_RATE,
//		UBX_CFG_RINV, UBX_CFG_RST, UBX_CFG_RXM,
//		UBX_CFG_SBAS, UBX_CFG_TMODE3,
//		UBX_CFG_TP5, UBX_CFG_USB, UBX_CFG_VALDEL,
//		UBX_CFG_VALGET, UBX_CFG_VALSET
//};

#define PORT_ID_SPI 4 //spi com port number for polling configs

#define UBX_PACKET_HEADER_SIZE 			6 //1(8) + 1(8) + 1(8) + 1(8) + 2(16)
#define UBX_PACKET_FOOTER_SIZE   2 //2 checksum bytes

typedef struct NeoGPSConfig_t {
	SPI_HandleTypeDef *spi_port;
	GPIO_TypeDef *cs_pin_port;
	uint16_t cs_pin;
} NeoGPSConfig_t;

typedef struct UBX_Packet_t {
	uint8_t class;
	uint8_t id;
	uint16_t length;
	uint16_t recv_counter;
	uint8_t *payload;
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

bool neom9n_begin(NeoGPSConfig_t *config, uint32_t max_wait);


#endif /* SRC_DRIVERS_NEOM9N_H_ */
