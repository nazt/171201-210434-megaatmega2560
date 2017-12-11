#include <Arduino.h>
#include <AIS_NB_IoT.h>
#include "CMMC_Interval.hpp"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#define ENABLE_AIS_NB_IOT 1

#include <NMEAGPS.h>
#include <GPSport.h>

static NMEAGPS  gps; // This parses the GPS characters

static int32_t gps_latitude = 0;
static int32_t gps_longitude = 0;
static int32_t gps_altitude_cm = 0;
static uint32_t gps_us;
static uint32_t bme680_tmp;
static uint32_t bme680_hum;
static uint32_t bme_gas_resistance_ohm;

static void doSomeWork( const gps_fix & fix );
static void doSomeWork( const gps_fix & fix )
{
  if (fix.valid.location) {
    gps_latitude = fix.latitudeL();
    gps_longitude = fix.longitudeL();
    gps_altitude_cm = fix.altitude_cm();
    gps_us = fix.dateTime_us();
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

Adafruit_BME680 bme; // I2C
#define SEALEVELPRESSURE_HPA (1013.25)

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
CMMC_RX_Parser parser(&Serial3);

bool flag_dirty = false;
char bbb[400];

static CMMC_MASTER_PACKET_T master_packet;

static void msg_recv(u8 * packet, u8 len) {
  Serial.println("RECV PACKET..");
  Serial.println();
  Serial.print(F("packet size= "));
  Serial.println(len);

  Serial.println("MASTER_PACKET..");
  Serial.print("master_packet size= ");
  Serial.println(sizeof(master_packet));
  if (len == sizeof(master_packet.packet)) {
    memcpy(&(master_packet.packet), packet, len);

    Serial.println(String("project = ") + master_packet.packet.project);
    Serial.println(String("version = ") + master_packet.packet.version);
    Serial.println(String("field1 = ")  + master_packet.packet.data.field1);
    Serial.println(String("field2 = ")  + master_packet.packet.data.field2);
    Serial.println(String("battery = ") + master_packet.packet.data.battery);
    Serial.println(String("myName= ") + master_packet.packet.data.myName);

    flag_dirty = true;
  }
}

void setup()
{
  Serial.begin(57600);
  Serial.println("Waiting NB-IoT first boot..");
  Serial3.begin(57600);
  gpsPort.begin(9600);
  Wire.begin();

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
  }
  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  Serial.println("BEGIN...");
#ifdef ENABLE_AIS_NB_IOT
  delay(5000); // wait nb-iot module boot
  // AISnb.debug = true;
  AISnb.setupDevice(serverPort);
  String ip1 = AISnb.getDeviceIP();
  Serial.println("Connected...");
  pingRESP pingR = AISnb.pingIP(serverIP);
#endif
  parser.on_command_arrived(&msg_recv);
}
String hexString;
signal  sig;
void loop()
{
  parser.process();
  GPSloop();
  if (flag_dirty) {

#ifdef ENABLE_AIS_NB_IOT
    sig = AISnb.getSignal();
    master_packet.nb_ber = sig.ber.toInt();
    master_packet.nb_rssi = sig.rssi.toInt();
    master_packet.nb_csq = sig.csq.toInt();
#endif 
    master_packet.gps_altitude_cm = gps_altitude_cm;
    master_packet.gps_latitude = gps_latitude;
    master_packet.gps_longitude = gps_longitude;
    master_packet.gps_us = gps_us;
    master_packet.temperature_c = bme680_tmp;
    master_packet.humidity_percent_rh = bme680_hum;
    master_packet.gas_resistance_ohm =  bme_gas_resistance_ohm;
    master_packet.cnt++;
    array_to_string((byte*)&master_packet, sizeof(master_packet), bbb);
#ifdef ENABLE_AIS_NB_IOT
    UDPSend res = AISnb.sendUDPmsg(serverIP, serverPort, String(bbb));
    if (res.status) {
      Serial.println("SEND OK"); 
      // AISnb.closeUDPSocket();
    }
    else {
      Serial.println("SEND FAILED.");
    }
#endif
    flag_dirty = false;
  }

  interval.every_ms(2L * 1000, []() {
    if (!bme.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    }
    bme680_tmp = bme.temperature*100;
    bme680_hum = bme.humidity*100;
    bme_gas_resistance_ohm = bme.gas_resistance; 
  });
}
