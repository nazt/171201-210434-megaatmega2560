#include <Arduino.h>
#include <AIS_NB_IoT.h>
#include "CMMC_Interval.hpp"

String serverIP = "103.212.181.167";
String serverPort = "55566";

AIS_NB_IoT AISnb;
AIS_NB_IoT_RES resp;

String udpData = "";
CMMC_Interval interval; 
#include <CMMC_RX_Parser.h>
#include "packet.h"
void array_to_string(byte array[], unsigned int len, char buffer[])
{
  for (unsigned int i = 0; i < len; i++)
  {
    byte nib1 = (array[i] >> 4) & 0x0F;
    byte nib2 = (array[i] >> 0) & 0x0F;
    buffer[i * 2 + 0] = nib1  < 0xA ? '0' + nib1  : 'A' + nib1  - 0xA;
    buffer[i * 2 + 1] = nib2  < 0xA ? '0' + nib2  : 'A' + nib2  - 0xA;
  }
  buffer[len * 2] = '\0';
}
String hexString = "";
long cnt = 0;
CMMC_RX_Parser parser(&Serial3);

bool flag_dirty = false;
char bbb[256];
CMMC_PACKET_T cmmc_packet;
void dump(const u8* data, size_t size) {
  for (size_t i = 0; i < size; i++) {
    Serial.print(data[i], HEX);
  }
}


void setup()
{
  AISnb.debug = true;
  Serial.begin(57600);
  Serial3.begin(57600);
  Serial.println("BEGIN...");
  AISnb.setupDevice(serverPort);
  String ip1 = AISnb.getDeviceIP();
  pingRESP pingR = AISnb.pingIP(serverIP);

  parser.on_command_arrived([](u8* packet, u8 len) {
    memcpy((uint8_t*)&cmmc_packet, packet, len);
    Serial.println(String("project = ") + cmmc_packet.project);
    Serial.println(String("version = ") + cmmc_packet.version);
    Serial.println(String("field1 = ") + cmmc_packet.data.field1);
    Serial.println(String("field2 = ") + cmmc_packet.data.field2);
    Serial.println(String("battery = ") + cmmc_packet.data.battery); 
    // Serial.print("cmmc_packet size= ");
    // Serial.println(sizeof(cmmc_packet));
    // dump((uint8_t*)&cmmc_packet, len); 
    array_to_string((byte*)packet, len+4, bbb);
    flag_dirty = true;
  });
}
void loop()
{
  parser.process();
  if (flag_dirty) {
    signal sig = AISnb.getSignal();
    hexString = String(bbb);
    String hexString = String(bbb);
    AISnb.sendUDPmsg(serverIP, serverPort, hexString); 
    flag_dirty = false;
  }
  // interval.every_ms(5L * 1000, []() {
  //   signal sig = AISnb.getSignal();
  //   //    Serial.print("csq: " + sig.csq);
  //   //    Serial.print("rssi: " + sig.rssi);
  //   //    Serial.print("ber: " + sig.ber);
  //   UDPSend udp;
  //   cnt++;
  //   // Send data in String
  //   udp = AISnb.sendUDPmsgStr(serverIP, serverPort, hexString);

  //   //udpDataHEX = AISnb.str2HexStr(udpData);

  //   // Send data in HexString
  //   //udp = AISnb.sendUDPmsg( serverIP, serverPort, udpDataHEX);
  // });

  // UDPReceive resp = AISnb.waitResponse();
}



