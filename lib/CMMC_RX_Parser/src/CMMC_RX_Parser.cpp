#include "CMMC_RX_Parser.h"

// typedef enum {
//   WAIT_STATE = 0,
//   CMD_STATE,
//   DATA_STATE
// } CMMC_SERIAL_CMD_STATE;

uint8_t START_BYTE_1 = 0x7e;
uint8_t START_BYTE_2 = 0x7f;
uint8_t STOP_BYTE_1 = 0x0d;
uint8_t STOP_BYTE_2 = 0x0a;

uint8_t CMMC_RX_Parser::_read_rx() {
  delayMicroseconds(100);
  return _serial->read();
}
void CMMC_RX_Parser::_parse(uint8_t data) {
  switch (_state) {
    case WAIT_STATE :
      if (data == START_BYTE_1) {
        // _packet.head[0] = data;
        _packet.data[_len++] = data;
        data = this->_read_rx();
        if (data == START_BYTE_2) {
          _packet.data[_len++] = data;
          _state = DATA_STATE;
        }
      }
      break;
    // case CMD_STATE :
    //   _packet.cmd = data;
    //   _state = DATA_STATE;
    //   break;
    case DATA_STATE :
      if (data == STOP_BYTE_1) { // found stop-byte1
        uint8_t data_next = this->_read_rx();
        if (data_next == STOP_BYTE_2) {  // found stop-byte2
          // we want stop byte.
          // so add tail bytes to user-space
          _packet.data[_len++] = data;
          _packet.data[_len++] = data_next;
          if (_user_on_data != NULL) { 
            memcpy(&_user_packet, &_packet, sizeof(_packet));
            _user_on_data((u8*)&_user_packet, _len); 
          }
          this->_len = 0;
          _state = WAIT_STATE;
          break;
        }
        else { // not stop-byte 2
          _packet.data[_len++] = data;
          _packet.data[_len++] = data_next;
        }
      } 
      else { // not any stop-byte
        _packet.data[_len++] = data;
      }
      break;
    default:
      break;
  }
}


void CMMC_RX_Parser::process() {
  while (_serial->available()) {
    uint8_t b = this->_read_rx();
    _parse(b);
  }
}