#include <Arduino.h>

typedef struct __attribute((__packed__)) {
  uint8_t from[6];
  uint8_t to[6];
  uint8_t type = 0;
  uint32_t battery = 0x00;
  uint32_t field1 = 0x00;
  uint32_t field2 = 0x00;
  uint32_t field3 = 0x00;   
  uint32_t field4 = 0x00;
  uint32_t field5 = 0x00;
  uint32_t field6 = 0x00;
  uint8_t nameLen = 0x00;
  char myName[15]; 
  uint32_t ms = 0;
  uint32_t sent_ms = 0;
  uint32_t sum = 0;
} CMMC_SENSOR_T; 

typedef struct __attribute((__packed__)) {
  uint8_t header[2] = {0xfa, 0xfb};
  uint8_t version = 9;
  uint8_t project = 8;
  uint8_t reserved[4];
  CMMC_SENSOR_T data;
  uint32_t sleepTime;
  uint32_t ms;
  uint32_t sum;
  uint8_t tail[2] = {0x0d, 0x0a};
} CMMC_PACKET_T; 


typedef struct __attribute((__packed__)) {
  uint8_t header[2] = {0xda, 0xdb};
  uint8_t version = 2;
  uint8_t project = 2;
  uint8_t reserved[4]; CMMC_PACKET_T packet;
  uint32_t temperature_c;
  uint32_t humidity_percent_rh;
  uint32_t sound_avg_db;   // Mic 
  uint32_t max_acc; // Mpu9250
  uint32_t acc_x;
  uint32_t acc_y;
  uint32_t acc_z;
  uint32_t gyro_x;
  uint32_t gyro_y;
  uint32_t gyro_z;
  uint32_t mag_x;
  uint32_t mag_y;
  uint32_t mag_z; 
  uint32_t nb_rssi; 
  uint32_t nb_csq; 
  uint32_t nb_ber; 
  uint32_t sum;
  uint8_t tail[2] = {0x0d, 0x0a};
} CMMC_MASTER_PACKET_T;