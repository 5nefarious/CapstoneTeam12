#include <global.h>
#include <Canbus.h>
#include <mcp2515_defs.h>
#include <mcp2515.h>
#include <defaults.h>

#define SUPPLY_VOLTAGE 5000
#define BAUD_RATE 9600
#define REFRESH_RATE 100
#define ADC_RESOLUTION 1024

#define MESSAGE_PRESSURE 0x631

#define GET_PRESSURE_FROM_VOLTAGE(X) (X * 1000 / SUPPLY_VOLTAGE - 40) * 11

int pressureSensorPin = A0;
long pressure_Pa;

void setup() {
  Serial.begin(BAUD_RATE);
  delay(1000);
  
  if (Canbus.init(CANSPEED_500))
    Serial.println("CAN bus initialized.");
  else
    Serial.println("CAN controller failed initialization!");
}

void loop() {
  int pressureSensorValue = analogRead(pressureSensorPin);
  long pressureSensorVoltage = (long) pressureSensorValue * SUPPLY_VOLTAGE / ADC_RESOLUTION;
  pressure_Pa = GET_PRESSURE_FROM_VOLTAGE(pressureSensorVoltage);

  tCAN message;
  message.id = MESSAGE_PRESSURE;
  message.header.rtr = 0;
  message.header.length = 4;
  memcpy(message.data, &pressure_Pa, 4);
  mcp2515_bit_modify(CANCTRL, (1 << REQOP2) | (1 << REQOP1) | (1 << REQOP0), 0);
  mcp2515_send_message(&message);

  Serial.print(pressureSensorValue);
  Serial.print(", ");
  Serial.print(pressureSensorVoltage);
  Serial.print(" mV, ");
  Serial.print(pressure_Pa);
  Serial.println(" Pa");

  delay(1000 / REFRESH_RATE);
}
