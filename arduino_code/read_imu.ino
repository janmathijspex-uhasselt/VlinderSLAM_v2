#include <Custom_Adafruit_BNO08x.h>
#include <Wire.h>

Adafruit_BNO08x bno;

#pragma pack(push,1)
struct ImuData {
  uint16_t header;
  
  uint32_t timestamp = 0;
  float ax, ay, az, gx, gy, gz;
};
#pragma pack(pop)

ImuData outData;

void setup() {  
  Serial.begin(115200);
  while (!Serial);

  outData.header = 0xABCD;

  while (true) {
      if (!bno.begin_I2C()) {
        // Serial.println("Kon geen BNO085 vinden! Opnieuw proberen...");
        delay(250);
      }
      else {
        break;
      }
  }

  Wire.setClock(250000);

  uint32_t interval_a = 5000;
  uint32_t interval_g = 5000;

  bno.enableReport(SH2_ACCELEROMETER, interval_a);
  bno.enableReport(SH2_GYROSCOPE_UNCALIBRATED, interval_g);

  // Serial.println("Start uitlezen!");
}

void loop() {
  /* Count print */
  /*
  static int last_t = 0;
  int current_t = millis();
  
  static int count_a = 0, count_g = 0;

  if (current_t - last_t > 1000) {
    Serial.print("count_a: "); Serial.print(count_a); Serial.print(" | count_g: "); Serial.println(count_g);

    count_a = 0;
    count_g = 0;
    last_t = current_t;
  }
  */

  static sh2_SensorValue_t sensorValue;
  if (bno.getSensorEvent(&sensorValue)) {

    if (sensorValue.sensorId == SH2_ACCELEROMETER) {       
      outData.ax = sensorValue.un.accelerometer.x;
      outData.ay = sensorValue.un.accelerometer.y;
      outData.az = sensorValue.un.accelerometer.z;

      outData.timestamp = sensorValue.timestamp32;

      /* Count print */
      // count_a++;

      /* Send data */
      // Serial.write((uint8_t*)&outData, sizeof(outData));

      /* Serial prints */
      // String s = "a: (x,y,z)=(" + (String)sensorValue.un.linearAcceleration.x + "," + sensorValue.un.linearAcceleration.y + "," + sensorValue.un.linearAcceleration.z + ") | status=" + sensorValue.status;
      // Serial.println(s);

      /* Serial plotter*/
      /*
      Serial.print("a_x:");
      Serial.print(outData.ax);
      Serial.print(",");
      Serial.print("a_y:");
      Serial.print(outData.ay);
      Serial.print(",");
      Serial.print("a_z:");
      Serial.println(outData.az);
      */
    }
    else if (sensorValue.sensorId == SH2_GYROSCOPE_UNCALIBRATED) {
      outData.gx = sensorValue.un.gyroscopeUncal.x;
      outData.gy = sensorValue.un.gyroscopeUncal.y;
      outData.gz = sensorValue.un.gyroscopeUncal.z;

      // outData.timestamp = sensorValue.timestamp32;

      /* Count print */
      // count_g++;

      /* Send data */
      Serial.write((uint8_t*)&outData, sizeof(outData));

      /* Serial prints */
      // String s = "g: (x,y,z)=(" + (String)sensorValue.un.linearAcceleration.x + "," + sensorValue.un.linearAcceleration.y + "," + sensorValue.un.linearAcceleration.z + ") | status=" + sensorValue.status;
      // Serial.println(s);

      /* Serial plotter*/
      /*
      Serial.print("g_x:");
      Serial.print(outData.gx);
      Serial.print(",");
      Serial.print("g_y:");
      Serial.print(outData.gy);
      Serial.print(",");
      Serial.print("g_z:");
      Serial.println(outData.gz);
      */
    }
  }
}
