#include <Arduino.h>
#include <AIS_NB_IoT.h>
#include "CMMC_Interval.hpp"

#define ENABLE_AIS_NB_IOT 1

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
long cnt = 0;
CMMC_RX_Parser parser(&Serial3);

bool flag_dirty = false;
char bbb[350];
void dump(const u8* data, size_t size) {
  array_to_string((byte*)data, size, bbb);
  // array_to_string((byte*)packet, len+4, buffer);
  // Serial.println(bbb); 
  // for (size_t i = 0; i < size; i++) {
  //   Serial.print(data[i], HEX);
  // }
}

//CMMC_PACKET_T cmmc_packet;
CMMC_MASTER_PACKET_T master_packet;

void setup()
{
  AISnb.debug = true;
  Serial.begin(57600);
  Serial3.begin(57600);
  Serial.println("BEGIN...");
  #ifdef ENABLE_AIS_NB_IOT
  AISnb.setupDevice(serverPort);
  String ip1 = AISnb.getDeviceIP();
  pingRESP pingR = AISnb.pingIP(serverIP);
  #endif 

  parser.on_command_arrived([](u8* packet, u8 len) {
    Serial.println("RECV PACKET..");
    Serial.println();
    Serial.print("packet size= "); 
    Serial.println(len); 
    // Serial.print("CMMC PACKET size= "); 
    // Serial.println(sizeof(cmmc_packet)); 
    // memcpy(&cmmc_packet, packet, len); 
    // dump(packet, len); 


    // memcpy((uint8_t*)&master_packet.packet, &packet, len); 

    Serial.println("MASTER_PACKET..");
    Serial.print("master_packet size= ");
    Serial.println(sizeof(master_packet));
    memcpy(&(master_packet.packet), packet, len); 

    Serial.println(String("project = ") + master_packet.packet.project);
    Serial.println(String("version = ") + master_packet.packet.version);
    Serial.println(String("field1 = ")  + master_packet.packet.data.field1);
    Serial.println(String("field2 = ")  + master_packet.packet.data.field2);
    Serial.println(String("battery = ") + master_packet.packet.data.battery); 

    // master_packet.packet = cmmc_packet; 
    // dump((uint8_t*)&master_packet, sizeof(master_packet)); 
    // for (int ii ; ii < sizeof(master_packet); ii++) { 
    //   Serial.print( ((uint8_t*) &master_packet)[ii], HEX); 
    // }
    // Serial.println(bbb);
    // delay(100);
    flag_dirty = true;
  });
} 
String hexString;
void loop()
{
  parser.process();
  if (flag_dirty) {
  #ifdef ENABLE_AIS_NB_IOT
    array_to_string((byte*)&master_packet, sizeof(master_packet), bbb); 
    // array_to_string((byte*)&(master_packet.packet), sizeof(master_packet.packet), bbb); 
    hexString = String(bbb);
    signal sig = AISnb.getSignal();
    AISnb.sendUDPmsg(serverIP, serverPort, hexString); 
  #endif
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



  #ifdef ENABLE_AIS_NB_IOT
  //  UDPReceive resp = AISnb.waitResponse();
  #endif
}



