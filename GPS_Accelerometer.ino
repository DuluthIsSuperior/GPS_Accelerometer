#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
SoftwareSerial ss(10, 11); // RX, TX
File dataFile;

const char decFormat[] PROGMEM = "%d,";
const char strFormat[] PROGMEM = "%s,";
const char strFormatNoComma[] PROGMEM = "%s";
const char accFormat[] PROGMEM = "a%s,";
const char noData[] PROGMEM = "a---.--,---.--,---.--,";
const char defaultUpdate[] PROGMEM = "--/--/---- --:--:--";
const char lastUpdatedFormat[] PROGMEM = "%s/%s/%d,%s:%s:%s";
const char prependZerosToString[] PROGMEM = "00%s";
const char prependZeroToString[] PROGMEM = "0%s";
const char prependNegativeZeroToString[] PROGMEM = "-0%s";
const char prependNegativeToString[] PROGMEM = "-%s";
const char appendString[] PROGMEM = "%s%s";

const int sdChipSelect = 7;

double oldLat = 0.0;
double oldLon = 0.0;
char lastUpdated[24];
const int decBufSize = 10;
const int numOfDecBufs = 6;
char decBufs[numOfDecBufs][decBufSize];
void setup() {
  // BEGIN LAMP TEST
  pinMode(53, OUTPUT);
  pinMode(49, OUTPUT);
  digitalWrite(53, HIGH);
  digitalWrite(49, HIGH);
  delay(2000);
  digitalWrite(53, HIGH);
  digitalWrite(49, HIGH);
  // END LAMP TEST
  
  Serial.begin(9600);
  while (!Serial) {
    delay(10);  // lets serial start up before proceeding
  }
  Serial.println(F("Booting up..."));

  if (!mpu.begin()) {
    Serial.println(F("Failed to find accelerometer; we are hanging here"));
    while (1) {}
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // can be MPU6050_RANGE_X_G where X can be 2, 4, 8, or 16 - the smaller the number, the more sensitive it is
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);  // MPU6050_RANGE_X_DEG where X = 250, 500, 1000, or 2000
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);  // MPU6050_BAND_X_HZ where X = 5, 10, 21, 44, 94, 184, 260 - used to filter out noise

  ss.begin(9600);

  if (!SD.begin(sdChipSelect)) {
    Serial.println(F("Could not find SD card"));
    digitalWrite(49, HIGH);
  }
  sprintf_P(lastUpdated, defaultUpdate);
  for (int i = 0; i < numOfDecBufs; i++) {
    clear(decBufs[i], decBufSize);
  }

  pinMode(2, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  Serial.println(F("Running"));
  delay(1000);  // gives time to read errors before proceeding
  digitalWrite(53, HIGH);
}

const int bufSize = 22 + 1; // + 1 for null terminator
char buf[bufSize];
int records = 0;
float xAccelerationOffset = 0.77f;
float yAccelerationOffset = 0.24f;
float zAccelerationOffset = -12.39f;
int retries = 0;
int retriesAllowed = 2000;
bool recording = false;
bool oldA6 = false;
void loop() {
  if (records == 0) {
    dataFile = SD.open("datalog.txt", FILE_WRITE);  // didn't like being initalized in setup() for some reason
  }

  digitalWrite(49, dataFile ? LOW : HIGH);
  if (!dataFile) {
    Serial.print(F("ERR -> "));
  } else if (recording) {
    Serial.print(F("REC -> "));
  } else {
    Serial.print(F("STP -> "));
  }

  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      if (gps.location.isValid()) {
        oldLat = gps.location.lat();
        oldLon = gps.location.lng();
      }
      if (gps.date.isValid() && gps.time.isValid()) {
        intToString(gps.date.month(), decBufs[0]);
        intToString(gps.date.day(), decBufs[1]);
        intToString(gps.time.hour(), decBufs[2]);
        intToString(gps.time.minute(), decBufs[3]);
        intToString(gps.time.second(), decBufs[4]);
        sprintf_P(lastUpdated, lastUpdatedFormat, decBufs[0], decBufs[1], gps.date.year(), decBufs[2], decBufs[3], decBufs[4]);
        for (int i = 0; i <= 4; i++) {
          clear(decBufs[i], decBufSize);
        }
      }
    }
  }
  
  sprintf_P(buf, strFormat, lastUpdated);
  printWriteAndClear(buf, dataFile, bufSize, false);
  
  dtostrf(oldLat, 1, 6, decBufs[0]);
  sprintf_P(buf, strFormat, decBufs[0]);
  printWriteAndClear(buf, dataFile, bufSize, false);
  clear(decBufs[0], decBufSize);
  
  dtostrf(oldLon, 1, 6, decBufs[0]);
  sprintf_P(buf, strFormat, decBufs[0]);
  printWriteAndClear(buf, dataFile, bufSize, false);
  clear(decBufs[0], decBufSize);

  sensors_event_t a, g, temp;
  if (mpu.getEvent(&a, &g, &temp)) {
    floatToString(a.acceleration.x + xAccelerationOffset, 2, decBufs[0]);
    sprintf_P(buf, accFormat, decBufs[0]);
    printWriteAndClear(buf, dataFile, bufSize, false);
    clear(decBufs[0], decBufSize);
    
    floatToString(a.acceleration.y + yAccelerationOffset, 2, decBufs[0]);
    sprintf_P(buf, strFormat, decBufs[0]);
    printWriteAndClear(buf, dataFile, bufSize, false);
    clear(decBufs[0], decBufSize);
    
    floatToString(a.acceleration.z + zAccelerationOffset, 2, decBufs[0]);
    sprintf_P(buf, strFormat, decBufs[0]);
    printWriteAndClear(buf, dataFile, bufSize, false);
    clear(decBufs[0], decBufSize);
  } else {
    sprintf_P(buf, noData);
    printWriteAndClear(buf, dataFile, bufSize, false);
    
    retries++;
    if (retries > retriesAllowed) {
      Serial.print(F("Accelerometer lost; we are hanging here"));
    }
  }

  sprintf_P(buf, decFormat, (digitalRead(2) == HIGH ? 1 : 2));  // if SPDT switch is ON I, then main 1 is selected
  printWriteAndClear(buf, dataFile, bufSize, false);

//  sprintf_P(buf, strFormat, digitalRead(A6) == HIGH ? "X" : " ");
//  printWriteAndClear(buf, dataFile, bufSize, false);
  bool newA6 = digitalRead(A6) == HIGH;
  if (newA6 && !oldA6) {
    oldA6 = true;
    recording = !recording;
  } else if (!newA6 && oldA6) {
    oldA6 = false;
  }
  sprintf_P(buf, strFormat, digitalRead(A7) == HIGH ? "X" : " ");
  printWriteAndClear(buf, dataFile, bufSize, false);
  sprintf_P(buf, strFormat, digitalRead(A8) == HIGH ? "X" : " ");
  printWriteAndClear(buf, dataFile, bufSize, false);
  sprintf_P(buf, strFormatNoComma, digitalRead(A9) == HIGH ? "X" : " ");
  printWriteAndClear(buf, dataFile, bufSize, false);
  if (dataFile) {
    dataFile.println();
  }
  sprintf_P(buf, strFormat, recording ? "|R" : "| ");
  printAndClear(buf, bufSize, true);

  records++;
  if (dataFile && records >= 10) {
    dataFile.flush();
    dataFile.close();
    records = 0;
  }
}

void clear(char *str, int size) {
  memset(&(str[0]), 0, size); // pointer to string, set size bytes to 0
}

void print(char *str, bool newLine) {
  if (newLine) {
    Serial.println(str);
  } else {
    Serial.print(str);
  }
}

void printAndClear(char *str, int size, bool newLine) {
  print(str, newLine);
  clear(str, size);
}

void printWriteAndClear(char *str, File file, int size, bool newLine) {
  print(str, newLine);
  if (recording && file) {
    if (newLine) {
      file.println(str);
    } else {
      file.print(str);
    }
  }
  clear(str, size);
}

void floatToString(float flt, int decimalPlaces, char *str) {
  char b[decBufSize];
  clear(b, decBufSize);
  dtostrf(flt < 0.00f ? (flt * -1.0f) : flt, 0, decimalPlaces, b);
  if (flt >= -9.99f && flt < 0.00f) {
    sprintf_P(str, prependNegativeZeroToString, b);
  } else if (flt >= 0.00f && flt <= 9.99f) {
    sprintf_P(str, prependZerosToString, b);
  } else if (flt >= -99.99f && flt <= -10.00f) {
    sprintf_P(str, prependNegativeToString, b);
  } else if (flt >= 10.00f && flt <= 99.99f) {
    sprintf_P(str, prependZeroToString, b);
  } else {
    sprintf_P(str, strFormatNoComma, b);
  }
}

void appendToString(char *dest, char *src) {
  char buf2[bufSize];
  clear(buf2, bufSize);
  sprintf_P(buf2, appendString, dest, src);
  sprintf_P(dest, strFormatNoComma, buf2);
}

void intToString(int integer, char *str) {
  char b[3];
  clear(b, 3);
  itoa(integer, b, 10);
  if (integer < 10) {
    sprintf(str, "0");
  }
  appendToString(str, b);
}
