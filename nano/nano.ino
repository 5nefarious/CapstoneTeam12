#include <global.h>
#include <Canbus.h>
#include <mcp2515_defs.h>
#include <mcp2515.h>
#include <defaults.h>
#include <limits.h>

// Constants
#define SUPPLY_VOLTAGE 5000
#define BAUD_RATE 9600
#define REFRESH_RATE 10
#define ADC_RESOLUTION 1024

#define PIN_PRESSURE_SENSOR   A0
#define PIN_ENCODER_CLOCK      7
#define PIN_ENCODER_OUTPUT     8
#define PIN_ENCODER_CS         9

#define NUM_ENCODER_ANGLE_BITS    9
#define NUM_ENCODER_STATUS_BITS   5

// Devices on network
#define DEVICE_HEAD   0x0
#define DEVICE_REAR   0x1
#define DEVICE_THIS DEVICE_REAR

// Message types
// Note: Restricted to 7 bits. MSB will be dropped.
#define MESSAGE_PAIN          0x01
#define MESSAGE_KEY_VALUE     0x02
#define MESSAGE_GET_KEY       0x03
#define MESSAGE_SET_KEY       0x04
#define MESSAGE_CAPTURE_KEY   0x05
#define MESSAGE_RESET_KEY     0x06

// Sensor subsystem
#define SENSOR_PROSTATE   0x00
#define SENSOR_SPHINCTER  0x01

#define NUM_SENSORS 2

// Parameter keys
#define KEY_ENABLED       0x00
#define KEY_VALUE         0x01
#define KEY_THRESHOLD_1   0x02
#define KEY_THRESHOLD_2   0x03
#define KEY_THRESHOLD_3   0x04

#define NUM_KEYS 5

#define GET_PRESSURE_FROM_VOLTAGE(X) (X * 1000 / SUPPLY_VOLTAGE - 40) * 11

static void send_key(byte sub, byte key);
static void send_pain(byte sub, byte level);
static void send_message(byte type, bool rtr, size_t len);

static tCAN message;
static long defaults[NUM_SENSORS][NUM_KEYS] = {
  { 0, 0, LONG_MAX, LONG_MAX, LONG_MAX },
  { 1, 0, 512, 640, 768 },
};
static long params[NUM_SENSORS][NUM_KEYS];
static byte flags;

static long last_encoder_angle = 0;
static long encoder_angle;
static byte encoder_status;

// Flags for keeping track of state
#define FLAG_SET       1
#define FLAG_SEND     (1 << 1)
#define FLAG_CAPTURE  (1 << 2)
#define FLAG_RESET    (1 << 3)

void setup() {
  
  // Initialize serial comm
  Serial.begin(BAUD_RATE);
  delay(1000);

  pinMode(PIN_ENCODER_CLOCK, OUTPUT);
  pinMode(PIN_ENCODER_OUTPUT, INPUT);
  pinMode(PIN_ENCODER_CS, OUTPUT);
  digitalWrite(PIN_ENCODER_CLOCK, LOW);

  memcpy(params, defaults, sizeof params);

  // Initialize CANBus
  if (Canbus.init(CANSPEED_500))
    Serial.println("CAN bus initialized.");
  else
    Serial.println("CAN controller failed initialization!");
}

void loop() {
  byte sub, key;
  long val;

  flags = 0; // clear all flags

  // Check for new messages
  if (mcp2515_check_message()) {
    if (mcp2515_get_message(&message)) {
      byte dev_id = message.id >> 7;
      byte msg_id = 0x7F & message.id;
      
      if (dev_id == DEVICE_HEAD) {
        switch (msg_id) {
        case MESSAGE_GET_KEY:
          flags |= FLAG_SEND;
          break;
        case MESSAGE_SET_KEY:
          flags |= FLAG_SET;
          break;
        case MESSAGE_CAPTURE_KEY:
          flags |= FLAG_CAPTURE;
          break;
        case MESSAGE_RESET_KEY:
          flags |= FLAG_RESET;
        }

        if (flags > 0) {
          // Extract message data
          sub = message.data[0];
          key = message.data[1];
          memcpy(&val, message.data + 2, 4);

          if (sub < 0 || sub >= NUM_SENSORS ||
              key < 0 || key >= NUM_KEYS)
            flags = 0;

          Serial.print(sub, HEX);
          Serial.print(", ");
          Serial.print(key, HEX);
          Serial.print(", ");
          Serial.println(val);
        }
      }
    }
  }

  if (params[SENSOR_PROSTATE][KEY_ENABLED]) {
    
    // Measure pressure in prostate
    int pressure_sensor_value = analogRead(PIN_PRESSURE_SENSOR);
    long pressure_sensor_voltage = (long) pressure_sensor_value
                                     * SUPPLY_VOLTAGE / ADC_RESOLUTION;
    params[SENSOR_PROSTATE][KEY_VALUE]
      = GET_PRESSURE_FROM_VOLTAGE(pressure_sensor_voltage);
  
    // Test pain thresholds in prostate
    for (int i = KEY_THRESHOLD_3; i >= KEY_THRESHOLD_1; i--) {
      if (params[SENSOR_PROSTATE][KEY_VALUE] > params[SENSOR_PROSTATE][i]) {
        send_pain(SENSOR_PROSTATE, i - KEY_THRESHOLD_1 + 1);
        break;
      }
    }
  }

  if (params[SENSOR_SPHINCTER][KEY_ENABLED]) {
    
    // Send CS pulse to notify sensor
    digitalWrite(PIN_ENCODER_CS, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_ENCODER_CS, LOW);

    // Read in sensor data over serial line
    byte ms_bits = shiftIn(PIN_ENCODER_OUTPUT, PIN_ENCODER_CLOCK, MSBFIRST);
    byte ls_bits = shiftIn(PIN_ENCODER_OUTPUT, PIN_ENCODER_CLOCK, MSBFIRST);
    unsigned short encoder_bits = (ms_bits << 8) | ls_bits;
    last_encoder_angle = encoder_angle;
    encoder_angle = encoder_bits >> 6;
    encoder_status = 0x3F & encoder_bits;
    params[SENSOR_SPHINCTER][KEY_VALUE] = encoder_angle;

    Serial.println(encoder_angle);
  
    // Test pain thresholds in sphincter
    for (int i = KEY_THRESHOLD_3; i >= KEY_THRESHOLD_1; i--) {
      if (params[SENSOR_SPHINCTER][KEY_VALUE] > params[SENSOR_SPHINCTER][i]) {
        send_pain(SENSOR_SPHINCTER, i - KEY_THRESHOLD_1 + 1);
        break;
      }
    }
  }
 
  if (flags & FLAG_SET)
    params[sub][key] = val;

  if (flags & FLAG_SEND)
    send_key(sub, key);

  if (flags & FLAG_CAPTURE)
    params[sub][key] = params[sub][KEY_VALUE];

  if (flags & FLAG_RESET)
    params[sub][key] = defaults[sub][key];

  delay(1000 / REFRESH_RATE);
}

static void send_key(byte sub, byte key) {
  message.data[0] = sub;
  message.data[1] = key;
  memcpy(message.data + 2, &(params[sub][key]), 4);
  send_message(MESSAGE_KEY_VALUE, false, 6);
}

static void send_pain(byte sub, byte level) {
  message.data[0] = sub;
  message.data[1] = level;
  send_message(MESSAGE_PAIN, false, 2);
}

static void send_message(byte type, bool rtr, size_t len) {
  message.id = (DEVICE_THIS << 7) | type;
  message.header.rtr = rtr;
  message.header.length = len;
  mcp2515_bit_modify(CANCTRL, (1 << REQOP2) | (1 << REQOP1) | (1 << REQOP0), 0);
  mcp2515_send_message(&message);
}
