#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <SPI.h>

#include "lepton_hw.h"

#include "encoders/as5048a/MagneticSensorAS5048A.h"
#include "comms/i2c/I2CCommander.h"

#define FIRMWARE_VERSION "1.0.0 by runger"


#define V_PSU 15.0f
#define V_LIMIT 10.0f
#define I2C_ADDRESS 0x42


//MagneticSensorAS5048A sensor = MagneticSensorAS5048A(PIN_SPI_SS);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5048_I2C);
BLDCDriver6PWM driver = BLDCDriver6PWM(PIN_PHASE_UH, PIN_PHASE_UL, PIN_PHASE_VH, PIN_PHASE_VL, PIN_PHASE_WH, PIN_PHASE_WL);
BLDCMotor motor = BLDCMotor(11);


// I2CCommander i2cCommander = I2CCommander();

// void onReceive(int numBytes) { i2cCommander.onReceive(numBytes); }
// void onRequest() { i2cCommander.onRequest(); }



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

  Wire.setClock(400000);
  sensor.init();

  // driver.voltage_power_supply = V_PSU;
  // driver.voltage_limit = V_LIMIT;
  // driver.pwm_frequency = 32000;
  // driver.init();

  // motor.linkDriver(&driver);
  // motor.linkSensor(&sensor);
  // motor.voltage_limit = V_LIMIT/2.0f;
  // motor.controller = MotionControlType::torque;
  // motor.torque_controller = TorqueControlType::voltage;
  // motor.motion_downsample = 4;
  // motor.init();
  // motor.initFOC();

  // Wire.setClock(400000);
  // Wire.begin(I2C_ADDRESS, true);
  // i2cCommander.addMotor(&motor);
  // i2cCommander.init(I2C_ADDRESS);
  // Wire.onReceive(onReceive);
  // Wire.onRequest(onRequest);
  // SimpleFOCDebug::print("I2C listening at address ");
  // SimpleFOCDebug::println(I2C_ADDRESS);

  ts = millis();
  SimpleFOCDebug::println("SimpleFOC Lepton initialized.");
  SimpleFOCDebug::println();
}





void loop() {
  // motor.loopFOC();
  // motor.move();
  delay(10);
  sensor.update();
  its++;
  if (millis() - ts > 1000) {
    ts = millis();
    SimpleFOCDebug::print("Angle: ");
    SimpleFOCDebug::println(sensor.getAngle());
    SimpleFOCDebug::print("Its/s: ");
    SimpleFOCDebug::println(its);
    its=0;
  }
}