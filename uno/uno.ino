#include <global.h>
#include <Canbus.h>
#include <mcp2515_defs.h>
#include <mcp2515.h>
#include <defaults.h>

// Constants
#define BAUD_RATE 9600
#define REFRESH_RATE 1000
#define INPUT_LENGTH 48

// Devices on network
// Can support up to 15 subsystems apart from the main controller.
#define DEVICE_HEAD 0x0
#define DEVICE_REAR 0x1
#define DEVICE_THIS DEVICE_HEAD

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

static const char *device_names[] = {
  "head",
  "rear",
};

static const char *sensor_names[] {
  "prostate",
  "sphincter",
};

static const char *key_names[] {
  "enabled",
  "value",
  "threshold1",
  "threshold2",
  "threshold3",
};

static tCAN message;
static char input[INPUT_LENGTH];
static int input_len = 0;
static char *command = input;

void setup() {
  // Initialize serial comm
  Serial.begin(BAUD_RATE);
  delay(1000);

  // Initialize CANBus
  if (Canbus.init(CANSPEED_500))
    Serial.println("CAN bus initialized. Reading CAN messages...");
  else
    Serial.println("CAN controller failed initialization!");
}

void loop() {
  byte type, dev, sub, key;
  long val;

  // Check for new messages
  if (mcp2515_check_message()) {
    if (mcp2515_get_message(&message)) {
      dev = message.id >> 7;
      type = 0x7F & message.id;
      
      switch (type) {
        case MESSAGE_PAIN:
          sub = message.data[0];
          val = message.data[1];
          Serial.print("Pain level ");
          Serial.print(val);
          Serial.print(" in ");
          Serial.print(device_names[dev]);
          Serial.print('.');
          Serial.println(sensor_names[sub]);
          break;
        case MESSAGE_KEY_VALUE:
          sub = message.data[0];
          key = message.data[1];
          memcpy(&val, message.data + 2, 4);
          Serial.print(device_names[dev]);
          Serial.print('.');
          Serial.print(sensor_names[sub]);
          Serial.print('.');
          Serial.print(key_names[key]);
          Serial.print(": ");
          Serial.println(val);
      }
    }
  }

  // Check for serial input
  if (Serial.available() > 0) {
    if (input_len >= INPUT_LENGTH - 1)
      input_len = 0;
    
    char incoming = Serial.read();
    if (incoming == '\n') {
      input[input_len] = '\0';

      command = strtok(input, " \t");
      char *subsystem_str = strtok(NULL, ".");
      char *key_str = strtok(NULL, " \t");
      val = atol(strtok(NULL, " \t\n"));

      bool error = false;
      type = 0;
      sub = 0;
      key = 0;

      if (strcmp(subsystem_str, "prostate") == 0) {
        sub = SENSOR_PROSTATE;
      } else if (strcmp(subsystem_str, "sphincter") == 0) {
        sub = SENSOR_SPHINCTER;
      } else {
        Serial.print("Unknown sensor/subsystem: ");
        Serial.println(subsystem_str);
        error = true;
      }

      if (strcmp(key_str, "enabled") == 0) {
        key = KEY_ENABLED;
      } else if (strcmp(key_str, "value") == 0) {
        key = KEY_VALUE;
      } else if (strcmp(key_str, "threshold1") == 0) {
        key = KEY_THRESHOLD_1;
      } else if (strcmp(key_str, "threshold2") == 0) {
        key = KEY_THRESHOLD_2;
      } else if (strcmp(key_str, "threshold3") == 0) {
        key = KEY_THRESHOLD_3;
      } else {
        Serial.print("Unknown key: ");
        Serial.println(key_str);
        error = true;
      }

      message.id = (DEVICE_THIS << 7);
      message.header.rtr = false;
      message.header.length = 2;
      message.data[0] = sub;
      message.data[1] = key;
  
      if (strcmp(command, "get") == 0) {
        type = MESSAGE_GET_KEY;
      } else if (strcmp(command, "set") == 0) {
        type = MESSAGE_SET_KEY;
        message.header.length += 4;
        memcpy(message.data + 2, &val, 4);
      } else if (strcmp(command, "capture") == 0) {
        type = MESSAGE_CAPTURE_KEY;
      } else if (strcmp(command, "reset") == 0) {
        type = MESSAGE_RESET_KEY;
      } else {
        Serial.print("Unknown command: ");
        Serial.println(command);
        error = true;
      }

      message.id |= type;

      if (!error) {
        mcp2515_bit_modify(CANCTRL, (1 << REQOP2) | (1 << REQOP1) | (1 << REQOP0), 0);
        mcp2515_send_message(&message);
      }

      input_len = 0;
      
    } else if (incoming >= 0)
      input[input_len++] = incoming;
      
  }

  delay(1000 / REFRESH_RATE);
}
