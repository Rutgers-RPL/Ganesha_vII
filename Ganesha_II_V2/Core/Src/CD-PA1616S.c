/*
 * PD1616S.c
 *
 *  Created on: Feb 21, 2025
 *      Author: mahirshah
 */


#include "CD-PA1616S.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define GPS_BUFFER_SIZE 128  // Buffer size for storing NMEA sentence


GPS_t gps;
uint8_t rx_data;
uint8_t rx_buffer[GPS_BUFFER_SIZE];
uint16_t rx_index = 0;

extern UART_HandleTypeDef huart8;  // Ensure this matches your UART

// Initialize UART for GPS reception
void GPS_Init(void) {
   HAL_UART_Receive_IT(GPS_UART, &rx_data, 1);
}

// UART Interrupt Callback (Stores Incoming Data)
void GPS_UART_Callback(UART_HandleTypeDef *huart) {
    if (huart->Instance == UART8) {
    	// Check if interrupt is from correct UART
        if (rx_data == '\n') {  // End of an NMEA sentence
            gps.nmea_sentence[rx_index] = '\0';  // Null-terminate the string
            gps.data_ready = 1;  // Mark as ready to process
            rx_index = 0;  // Reset buffer index
        } else {
            gps.nmea_sentence[rx_index++] = rx_data;
            if (rx_index >= GPS_BUFFER_SIZE) {
                rx_index = 0;  // Prevent buffer overflow
            }
        }
        HAL_UART_Receive_IT(GPS_UART, &rx_data, 1);  // Restart reception
    }
}

// Process GPS data using blocking `HAL_UART_Receive()`
GPS_t* GPS_Process(void) {
    memset(rx_buffer, 0, GPS_BUFFER_SIZE);  // Clear buffer

    // ✅ Read an entire NMEA sentence at once
    HAL_UART_Receive(&huart8, rx_buffer, GPS_BUFFER_SIZE, HAL_MAX_DELAY);

    // ✅ Ensure it's a valid NMEA sentence (must start with '$')
    if (rx_buffer[0] == '$') {
        strncpy(gps.nmea_sentence, (char*)rx_buffer, GPS_BUFFER_SIZE);  // Copy data
        gps.data_ready = 1;  // Mark as ready to process
    } else {
        gps.data_ready = 0;  // Ignore invalid data
        return NULL;
    }

    if (gps.data_ready) {
        GPS_ParseNMEA(&gps);
        gps.data_ready = 0;  // Reset flag after processing
        return &gps;  // Return pointer to updated GPS data
    }

    return NULL;  // No new data available
}


// Parse NMEA Sentence (Only Extracts Latitude & Longitude)
void GPS_ParseNMEA(GPS_t *gps) {
    if (strstr(gps->nmea_sentence, "$GPGGA") != NULL) {
        char *token;
        char buffer[GPS_BUFFER_SIZE];
        strcpy(buffer, gps->nmea_sentence);  // Copy to avoid modifying original

        token = strtok(buffer, ",");  // $GPGGA
        token = strtok(NULL, ",");    // UTC Time (ignored)

        token = strtok(NULL, ",");    // Latitude
        if (token) gps->latitude = atof(token) / 100.0;  // Convert to degrees

        token = strtok(NULL, ",");    // N/S Indicator
        if (token) gps->lat_dir = token[0];

        token = strtok(NULL, ",");    // Longitude
        if (token) gps->longitude = atof(token) / 100.0;  // Convert to degrees

        token = strtok(NULL, ",");    // E/W Indicator
        if (token) gps->lon_dir = token[0];

    }
}
