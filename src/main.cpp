#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>

#include "lepton_hw.h"

#include "encoders/sc60228/MagneticSensorSC60228.h"

#define FIRMWARE_VERSION "1.0.0 by runger"


// MagneticSensorSC60228 sensor = MagneticSensorSC60228(PIN_SPI_nCS);


long ts;      // timestamp
int its = 0;  // loop iterations per second

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
  delay(3000); // wait for serial monitor to be ready
  //while(!Serial);
  SimpleFOCDebug::println(" __ _                 _        ___  ___  ___\n/ _(_)_ __ ___  _ __ | | ___  / __\\/___\\/ __\\\n\\ \\| | '_ ` _ \\| '_ \\| |/ _ \\/ _\\ //  // /\n_\\ \\ | | | | | | |_) | |  __/ /  / \\_// /___\n\\__/_|_| |_| |_| .__/|_|\\___\\/   \\___/\\____/\n               |_|\n   __            _                      ____\n  / /  ___ _ __ | |_ ___  _ __   __   _|___ \\\n / /  / _ \\ '_ \\| __/ _ \\| '_ \\  \\ \\ / / __) |\n/ /__|  __/ |_) | || (_) | | | |  \\ V / / __/\n\\____/\\___| .__/ \\__\\___/|_| |_|   \\_/ |_____|\n          |_|\n");
  SimpleFOCDebug::print("by Valentine, firmware version ");
  SimpleFOCDebug::println(FIRMWARE_VERSION);
  SimpleFOCDebug::print("MCU Speed: ");
  SimpleFOCDebug::println((int)SystemCoreClock);
  SimpleFOCDebug::println();

  // sensor.init();

  ts = millis();
  SimpleFOCDebug::println("SimpleFOC Lepton initialized.");
}





void loop() {
  // sensor.update();
  its++;
  if (millis() - ts > 1000) {
    ts = millis();
    // SimpleFOCDebug::print("Angle: ");
    // SimpleFOCDebug::println(sensor.getAngle());
    SimpleFOCDebug::print("Iterations/s: ");
    SimpleFOCDebug::println(its);
    its=0;
  }
  delay(10);
}