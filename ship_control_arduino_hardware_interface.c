#include <Adafruit_GPS.h>

// GPS模块
Adafruit_GPS GPS(&Serial1);

void setup() {
  Serial.begin(9600);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
}

void loop() {
  // 读取GPS
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) return;
    Serial.print("GPS,");
    Serial.print(GPS.latitude, 4); Serial.print(",");
    Serial.println(GPS.longitude, 4);
  }
  
  // 读取红外传感器
  int infrared = analogRead(A0);
  Serial.print("INFRARED,");
  Serial.println(infrared);
  
  delay(1000);
}