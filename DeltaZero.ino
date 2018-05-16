#include <OneWire.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <SD.h>

OneWire oneWire(3);
DallasTemperature tempSensors(&oneWire);
DeviceAddress intTemp, ambTemp;

const unsigned char UBX_HEADER[] = {0xB5, 0x62};
const unsigned char NAV_POSLLH_HEADER[] = {0x01, 0x02};
const unsigned char NAV_STATUS_HEADER[] = {0x01, 0x03};
const unsigned char NAV_TIMEUTC_HEADER[] = {0x01, 0x21};

enum _ubxMsgType {
  MT_NONE,
  MT_NAV_POSLLH,
  MT_NAV_STATUS,
  MT_NAV_TIMEUTC
};

struct NAV_POSLLH {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  long lon;
  long lat;
  long alt;
  long altMSL;
  unsigned long hAcc;
  unsigned long vAcc;
};

struct NAV_STATUS {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  unsigned char gpsFix;
  char flags;
  char fixStat;
  char flags2;
  unsigned long ttff;
  unsigned long msss;
};

struct NAV_TIMEUTC {
  unsigned char cls;
  unsigned char id;
  unsigned short len;
  unsigned long iTOW;
  unsigned long tAcc;
  signed long nano;
  unsigned short year;
  unsigned char month;
  unsigned char day;
  unsigned char hour;
  unsigned char minute;
  unsigned char second;
  unsigned char valid;
};

union UBXMessage {
  NAV_POSLLH pos;
  NAV_STATUS status;
  NAV_TIMEUTC time;
};

UBXMessage ubxMessage;

void calcChecksum(unsigned char* CK, int msgSize) {
  memset(CK, 0, 2);
  for (int i = 0; i < msgSize; i++) {
    CK[0] += ((unsigned char*)(&ubxMessage))[i];
    CK[1] += CK[0];
  }
}

boolean compareMsgHeader(const unsigned char* msgHeader) {
  unsigned char* ptr = (unsigned char*)(&ubxMessage);
  return ptr[0] == msgHeader[0] && ptr[1] == msgHeader[1];
}

int processGPS() {
  static int fpos = 0;
  static unsigned char checksum[2];

  static byte currentMsgType = MT_NONE;
  static int payloadSize = sizeof(UBXMessage);

  while (Serial1.available()) {
    byte c = Serial1.read();

    if (fpos < 2) {
      if (c == UBX_HEADER[fpos]) fpos++;
      else fpos = 0;
    } else {
      if ((fpos - 2) < payloadSize)
        ((unsigned char*)(&ubxMessage))[fpos - 2] = c;

      fpos++;

      if (fpos == 4) {
        if (compareMsgHeader(NAV_POSLLH_HEADER)) {
          currentMsgType = MT_NAV_POSLLH;
          payloadSize = sizeof(NAV_POSLLH);
        } else if (compareMsgHeader(NAV_STATUS_HEADER)) {
          currentMsgType = MT_NAV_STATUS;
          payloadSize = sizeof(NAV_STATUS);
        } else if (compareMsgHeader(NAV_TIMEUTC_HEADER)) {
          currentMsgType = MT_NAV_TIMEUTC;
          payloadSize = sizeof(NAV_TIMEUTC);
        } else {
          fpos = 0;
          continue;
        }
      }

      if (fpos == (payloadSize + 2)) calcChecksum(checksum, payloadSize);
      else if (fpos == (payloadSize + 3)) {
        if (c != checksum[0]) fpos = 0;
      } else if (fpos == (payloadSize + 4)) {
        fpos = 0;
        if (c == checksum[1]) return currentMsgType;
      } else if (fpos > (payloadSize + 4)) fpos = 0;
    }
  }

  return MT_NONE;
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  tempSensors.begin();

  if (!tempSensors.getAddress(intTemp, 0)) {
    Serial.println("Could not find IntTemp");
    return setup();
  }

  if (!tempSensors.getAddress(ambTemp, 1)) {
    Serial.println("Could not find AmbTemp");
    return setup();
  }

  tempSensors.setResolution(intTemp, 8);
  tempSensors.setResolution(ambTemp, 8);

  if (!SD.begin(53)) {
    Serial.println("Could not initialize SD card");
    return setup();
  }

  pinMode(2, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(2), incrementRadiationCount, FALLING);
}

long lat, lon, alt;
char fixStatus, utcTime[15];
unsigned long lastUpdate, lastGpsPacket = millis();;
volatile unsigned int radiationCount = 0;

void loop() {
  int msgType = processGPS();

  if (msgType == MT_NAV_POSLLH) {
    lat = ubxMessage.pos.lat;
    lon = ubxMessage.pos.lon;
    alt = ubxMessage.pos.altMSL;
  } else if (msgType == MT_NAV_STATUS) {
    fixStatus = ubxMessage.status.gpsFix | (ubxMessage.status.flags & B00000001) << 3;
  } else if (msgType == MT_NAV_TIMEUTC) {
    sprintf(utcTime, "%02d%02d%04d%02d%02d%02d", ubxMessage.time.day, ubxMessage.time.month, ubxMessage.time.year, ubxMessage.time.hour, ubxMessage.time.minute, ubxMessage.time.second);
  }

  if (msgType != MT_NONE) lastGpsPacket = millis();

  if (millis() - lastUpdate >= 1000) {
    if (millis() - lastGpsPacket > 10000) {
      fixStatus = 0;
      Serial.println("*** WARNING *** Last GPS packet received >10s ago");
    }

    tempSensors.requestTemperatures();
    float intTempC = tempSensors.getTempC(intTemp);
    float ambTempC = tempSensors.getTempC(ambTemp);

    unsigned long cpm = radiationCount * (60000 / (millis() - lastUpdate));
    radiationCount = 0;

    Serial.print("$MOD");
    Serial.print(fixStatus, DEC);
    Serial.print(" LAT");
    Serial.print(lat / 10000000.0f, 8);
    Serial.print(" LON");
    Serial.print(lon / 10000000.0f, 8);
    Serial.print(" ALT");
    Serial.print(alt / 1000.0f, 8);
    Serial.print(" UTC");
    Serial.print(utcTime);
    Serial.print(" INT");
    Serial.print(intTempC);
    Serial.print(" AMB");
    Serial.print(ambTempC);
    Serial.print(" RCM");
    Serial.print(cpm);
    Serial.println();

    File file = SD.open("log.txt", FILE_WRITE);
    if (file) {
      file.print("$MOD");
      file.print(fixStatus, DEC);
      file.print(" LAT");
      file.print(lat / 10000000.0f, 8);
      file.print(" LON");
      file.print(lon / 10000000.0f, 8);
      file.print(" ALT");
      file.print(alt / 1000.0f, 8);
      file.print(" UTC");
      file.print(utcTime);
      file.print(" INT");
      file.print(intTempC);
      file.print(" AMB");
      file.print(ambTempC);
      file.print(" RCM");
      file.print(cpm);
      file.println();
      file.close();
    } else setup();

    lastUpdate = millis();
  }
}

void incrementRadiationCount() {
  radiationCount++;
}

