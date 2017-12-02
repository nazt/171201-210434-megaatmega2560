#ifndef CMMC_RX_Parser_H
#define CMMC_RX_Parser_H

#include <Arduino.h>

#define DATA_BUFFER 	SERIAL_RX_BUFFER_SIZE

typedef enum {
  WAIT_STATE = 0,
  DATA_STATE
} CMMC_SERIAL_CMD_STATE;


typedef struct __attribute((__packed__)) {
	uint8_t  data[100];
  uint16_t len;
} CMMC_SERIAL_PACKET_T;

typedef void(*callback_t)(u8* packet, u8 len);


class CMMC_RX_Parser
{
    public:
      // constructor
      CMMC_RX_Parser(Stream *s) { 
      	this->_serial = s;
      }

      ~CMMC_RX_Parser() { } 
      void _parse(uint8_t data);
      void process();
      void on_command_arrived(callback_t cb) {
        if (cb != NULL) {
          this->_user_on_data = cb; 
        } 
      }

    private:
      uint16_t _len = 0;
    	Stream *_serial;
    	CMMC_SERIAL_CMD_STATE _state;
    	CMMC_SERIAL_PACKET_T _packet;
      CMMC_SERIAL_PACKET_T _user_packet;
      callback_t _user_on_data = NULL;
      uint8_t _read_rx();
};

#endif //CMMC_RX_Parser_H
