#include <Arduino.h>
#include <AIS_NB_IoT.h>
#include "CMMC_Interval.hpp"

#define ENABLE_AIS_NB_IOT 1

#include <NMEAGPS.h>
#include <GPSport.h>

static NMEAGPS  gps; // This parses the GPS characters

static int32_t gps_latitude = 0;
static int32_t gps_longitude = 0;
static int32_t gps_altitude_cm = 0;
static uint32_t gps_us;

static void doSomeWork( const gps_fix & fix );
static void doSomeWork( const gps_fix & fix )
{

  if (fix.valid.location) {
    gps_latitude = fix.latitudeL();
    gps_longitude = fix.longitude();
    gps_altitude_cm = fix.altitude_cm();
    gps_us =  fix.dateTime_us();
  } 
} // doSomeWork
static void GPSloop();
static void GPSloop()
{
  while (gps.available( gpsPort ))
    doSomeWork( gps.read() );

} // GPSloop

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
char bbb[400];
void dump(const u8* data, size_t size) {
  array_to_string((byte*)data, size, bbb);
  // array_to_string((byte*)packet, len+4, buffer);
  // Serial.println(bbb);
  // for (size_t i = 0; i < size; i++) {
  //   Serial.print(data[i], HEX);
  // }
}

CMMC_MASTER_PACKET_T master_packet;

void setup()
{
  Serial.begin(57600);
  Serial.println("Waiting NB-IoT first boot..");
  delay(5000); // wait nb-iot module boot
  AISnb.debug = false;
  Serial3.begin(57600);
  gpsPort.begin(9600);
  Serial.println("BEGIN...");
#ifdef ENABLE_AIS_NB_IOT
  AISnb.setupDevice(serverPort);
  String ip1 = AISnb.getDeviceIP();
  Serial.println("Connected...");
  pingRESP pingR = AISnb.pingIP(serverIP);
#endif

  parser.on_command_arrived([](u8 * packet, u8 len) {
    Serial.println("RECV PACKET..");
    Serial.println();
    Serial.print("packet size= ");
    Serial.println(len);

    Serial.println("MASTER_PACKET..");
    Serial.print("master_packet size= ");
    Serial.println(sizeof(master_packet));
    memcpy(&(master_packet.packet), packet, len);

    Serial.println(String("project = ") + master_packet.packet.project);
    Serial.println(String("version = ") + master_packet.packet.version);
    Serial.println(String("field1 = ")  + master_packet.packet.data.field1);
    Serial.println(String("field2 = ")  + master_packet.packet.data.field2);
    Serial.println(String("battery = ") + master_packet.packet.data.battery);

    flag_dirty = true;
  });
}
String hexString;
signal  sig;
void loop()
{
  GPSloop();
  parser.process();
  if (flag_dirty) {
    sig = AISnb.getSignal();
    master_packet.gps_altitude_cm = gps_altitude_cm;
    master_packet.gps_latitude = gps_latitude;
    master_packet.gps_longitude = gps_longitude;
    master_packet.gps_us = gps_us;
    master_packet.nb_ber = sig.ber.toInt();
    master_packet.nb_rssi = sig.rssi.toInt();
    master_packet.nb_csq = sig.csq.toInt();
    array_to_string((byte*)&master_packet, sizeof(master_packet), bbb);
    UDPSend res = AISnb.sendUDPmsg(serverIP, serverPort, String(bbb));
    if (res.status) {
      Serial.println("SEND OK"); 
    }
    else {
      Serial.println("SEND FAILED.");
    }
    flag_dirty = false;
  }

  interval.every_ms(5L * 1000, []() { });
  UDPReceive resp = AISnb.waitResponse();

}



