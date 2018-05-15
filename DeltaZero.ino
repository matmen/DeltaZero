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

const PROGMEM String gpsFixStates[] = {"No Fix", "Dead Reckoning", "2D", "3D", "GPS + Dead Reckoning", "Time Only"};

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
}

long lat, lon, alt;

void loop() {
  int msgType = processGPS();

  if (msgType == MT_NAV_POSLLH) {
    Serial.print("Lat: ");
    Serial.println(ubxMessage.pos.lat / 10000000.0f, 10);
    Serial.print("Lon: ");
    Serial.println(ubxMessage.pos.lon / 10000000.0f, 10);
    Serial.print("Alt (above MSL, m): ");
    Serial.println(ubxMessage.pos.altMSL / 1000.0f);
  } else if (msgType == MT_NAV_STATUS) {
    Serial.print("Fix status: ");
    Serial.print(gpsFixStates[(int)ubxMessage.status.gpsFix]);
    Serial.print(" (");
    Serial.print(ubxMessage.status.flags & 1 ? "Valid" : "Invalid");
    Serial.println(")");
  } else if (msgType == MT_NAV_TIMEUTC) {
    char timeString[20];
    sprintf(timeString, "%02d.%02d.%04d %02d:%02d:%02d", ubxMessage.time.day, ubxMessage.time.month, ubxMessage.time.year, ubxMessage.time.hour, ubxMessage.time.minute, ubxMessage.time.second);
    Serial.print("Time (UTC): ");
    Serial.println(timeString);
    Serial.println();
  }

}

