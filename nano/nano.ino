#include <global.h>
#include <Canbus.h>
#include <mcp2515_defs.h>
#include <mcp2515.h>
#include <defaults.h>

#define SUPPLY_VOLTAGE 5
#define REFRESH_RATE 100
#define ADC_RESOLUTION 1024

#define MESSAGE_PRESSURE 0x631

float get_pressure_from_voltage(float v);

union Measure {
  float f;
  byte b[4];
};

int pressureSensorPin = A0;
Measure pressure_kPa;

void setup() {
  if (Canbus.init(CANSPEED_500))
    Serial.println("CAN bus initialized.");
  else
    Serial.println("CAN controller failed initialization!");
}

void loop() {
  int pressureSensorValue = analogRead(pressureSensorPin);
  float pressureSensorVoltage = pressureSensorValue * SUPPLY_VOLTAGE / (float) ADC_RESOLUTION;
  pressure_kPa.f = get_pressure_from_voltage(pressureSensorVoltage);

  tCAN message;
  message.id = MESSAGE_PRESSURE;
  message.header.rtr = 0;
  message.header.length = 4;
  for (int i = 0; i < 4; i++)
    message.data[i] = pressure_kPa.b[i];
  mcp2515_bit_modify(CANCTRL, (1 << REQOP2) | (1 << REQOP1) | (1 << REQOP0), 0);
  mcp2515_send_message(&message);
  
  delay(1000 / REFRESH_RATE);
}

float get_pressure_from_voltage(float voltage) {
  return (voltage / SUPPLY_VOLTAGE - 0.04) / 0.09;
}
