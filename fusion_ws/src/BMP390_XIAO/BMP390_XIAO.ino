/***************************************************************************
  I2C Example for BMP390 on Seeeduino XIAO @ 50 Hz
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for USB serial

  Serial.println("BMP390 + Seeeduino XIAO (I2C) @ 50 Hz");

  // Initialize I²C
  Wire.begin();  
  // If your XIAO uses non-standard pins for SDA/SCL, 
  // you can specify them like: Wire.setSDA(6); Wire.setSCL(7);

  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a valid BMP3 sensor (I2C) - check wiring!");
    while (1) { delay(10); }
  }

  // Configure sensor
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);  // 50 Hz
  delay(100);  // Let sensor stabilize
}

void loop() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
  } else {
    float altitude_m = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    Serial.println(altitude_m, 6);  // Print altitude in meters
  }

  // ~20 ms delay => ~50 Hz
  delay(20);
}
