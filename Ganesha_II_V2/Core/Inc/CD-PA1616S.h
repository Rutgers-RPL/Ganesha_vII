/*
 * CD-PA1616S.h
 *
 *  Created on: Feb 21, 2025
 *      Author: Mahir Shah
 */

#ifndef INC_CD_PA1616S_H_
#define INC_CD_PA1616S_H_

#include "stm32h7xx_hal.h" /* Needed for UART */

// Defines



#define GPS_UART &huart8  // Change to match your UART instance
#define GPS_BUFFER_SIZE 128  // Reduced buffer size

// GPS Data Structure (Only Latitude & Longitude)
typedef struct {
    char nmea_sentence[GPS_BUFFER_SIZE];  // Raw NMEA sentence
    uint8_t data_ready;  // Flag indicating new data available

    // Parsed GPS data
    float latitude;
    char lat_dir;
    float longitude;
    char lon_dir;
} GPS_t;

// Function Declarations
void GPS_Init(void);
GPS_t* GPS_Process(void);
void GPS_UART_Callback(UART_HandleTypeDef *huart);
void GPS_ParseNMEA(GPS_t *gps);





#endif /* INC_CD_PA1616S_H_ */
