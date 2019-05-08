#include <limits.h>

#include <global.h>
#include <Canbus.h>
#include <mcp2515_defs.h>
#include <mcp2515.h>
#include <defaults.h>

#include <SoftwareSerial.h>
#include <Adafruit_Soundboard.h>

#include <SPI.h>
#include <SD.h>

#include <VetMed_utils.h>

#define DEVICE_THIS DEVICE_HEAD

// Constants
#define SFX_BAUD_RATE 9600

#define PIN_SFX_RESET   4
#define PIN_SFX_TX      5
#define PIN_SFX_RX      6
#define PIN_SFX_ACT     7
//#define PIN_SFX_VOLDN   8
//#define PIN_SFX_VOLUP   9

#define PIN_JOY_UP      A1
#define PIN_JOY_DOWN    A3
#define PIN_JOY_LEFT    A2
#define PIN_JOY_RIGHT   A5
#define PIN_JOY_CLICK   A4

#define PIN_SD_CS       9

#include "config.h"

//static const char *const device_names[] = {
//  "head",
//  "rear",
//};
//
//static const char *const sensor_names[] = {
//  "prostate",
//  "anus",
//};
//
//static const char *const key_names[] = {
//  "status",
//  "value",
//  "threshold1",
//  "threshold2",
//  "threshold3",
//};
//
//static const char *const status_names[] = {
//  "off",
//  "on",
//  "log",
//};

static SoftwareSerial ss = SoftwareSerial(PIN_SFX_TX, PIN_SFX_RX);
static Adafruit_Soundboard sfx = Adafruit_Soundboard(&ss, NULL, PIN_SFX_RESET);

static tCAN message;

static char input[INPUT_LENGTH];
static int input_len = 0;
static char *command = input;

static bool sfx_available = false;
static short current_track = -1;
static short play_track = -1;

static File painlog;
static File datalog;

static byte last_pain = 0;
static unsigned long last_time = 0;

void setup() {
  // Initialize serial comm with computer
  Serial.begin(BAUD_RATE);
  delay(1000);

  // Initialize serial comm with sfx board
  ss.begin(SFX_BAUD_RATE);
  if (sfx.reset()) {
    Serial.println(F("SFX board found!"));
    sfx_available = true;

    uint8_t files = sfx.listFiles();
    Serial.print(F("Audio FX Files: "));
    Serial.println(files);
    for (uint8_t f = 0; f < files; f++) {
      Serial.print(f);
      Serial.print('\t'); Serial.print(sfx.fileName(f));
      Serial.print('\t'); Serial.println(sfx.fileSize(f));
    }
  } else
    Serial.println(F("SFX board offline..."));

  pinMode(PIN_SFX_ACT, INPUT_PULLUP);
//  pinMode(PIN_SFX_VOLUP, OUTPUT);
//  pinMode(PIN_SFX_VOLDN, OUTPUT);

  pinMode(PIN_JOY_UP, INPUT_PULLUP);
  pinMode(PIN_JOY_DOWN, INPUT_PULLUP);
  pinMode(PIN_JOY_LEFT, INPUT_PULLUP);
  pinMode(PIN_JOY_RIGHT, INPUT_PULLUP);
  pinMode(PIN_JOY_CLICK, INPUT_PULLUP);

  pinMode(PIN_SD_CS, OUTPUT);

  if (SD.begin(PIN_SD_CS)) {
    Serial.println(F("SD card found!"));
  } else
    Serial.println(F("No SD card detected."));

  // Initialize CANBus
  if (Canbus.init(CANSPEED_500))
    Serial.println(F("CAN bus initialized. Reading CAN messages..."));
  else
    Serial.println(F("CAN controller failed initialization!"));

  last_time = millis();
}

void loop() {
  byte type, dev, sub, key;
  long val;

  // Check for new messages
  if (mcp2515_check_message()) {
    if (mcp2515_get_message(&message)) {
      unsigned long arrival_time = millis();
      dev = message.id >> 7;
      type = 0x7F & message.id;
      const char *str;

      // Handle different message types
      switch (type) {
        case MESSAGE_PAIN:
          sub = message.data[0];
          val = message.data[1];
          if (val > last_pain) {
            last_pain = val;
            painlog = SD.open(F("painlog.csv"), FILE_WRITE);
            if (painlog) {
              painlog.print(arrival_time);
              painlog.print(F(", "));
            }
            str = retreive_str(device_strs, dev);
            Serial.print(F("pain "));
            Serial.print(str);
            Serial.print('.');
            if (painlog) {
              painlog.print(str);
              painlog.print('.');
            }
            str = retreive_str(sensor_strs, sub);
            Serial.print(str);
            Serial.print(F(": "));
            Serial.println(val);
            if (painlog) {
              painlog.print(str);
              painlog.print(F(", "));
              painlog.println(val);
              painlog.close();
            }
          }
          break;
        case MESSAGE_KEY_VALUE:
          sub = message.data[0];
          key = message.data[1];
          memcpy(&val, message.data + 2, 4);
//          Serial.print(device_names[dev]);
//          Serial.print('.');
//          Serial.print(sensor_names[sub]);
//          Serial.print('.');
//          Serial.print(key_names[key]);
//          Serial.print(F(": "));
//          if (key == KEY_STATUS)
//            Serial.println(status_names[val]);
//          else
//            Serial.println(val);
          datalog = SD.open(F("datalog.csv"), FILE_WRITE);
          if (datalog) {
            datalog.print(arrival_time);
            datalog.print(F(", "));
          }
          str = retreive_str(device_strs, dev);
          Serial.print(str);
          Serial.print('.');
          if (datalog) {
            datalog.print(str);
            datalog.print('.');
          }
          str = retreive_str(sensor_strs, sub);
          Serial.print(str);
          Serial.print('.');
          if (datalog) {
            datalog.print(str);
            datalog.print('.');
          }
          str = retreive_str(key_strs, key);
          Serial.print(str);
          Serial.print(F(": "));
          if (datalog) {
            datalog.print(str);
            datalog.print(F(", "));
            datalog.println(val);
            datalog.close();
          }
          if (key == KEY_STATUS)
            Serial.println(retreive_str(status_strs, val));
          else
            Serial.println(val);
      }
    }
  }

  // Check for serial input
  while (Serial.available() > 0) {
    char incoming = Serial.read();
    Serial.print(incoming);
    
    if (input_len >= INPUT_LENGTH - 1)
      input_len = 0;
    else if (incoming >= 0)
      // Copy characters into the input buffer
      input[input_len++] = incoming;

    // Detect newline and interpret command
    if (incoming == '\n') {

      // Split input buffer into command and arguments 
      command = strtok(input, " \t\n");
      char *sub_str = strtok(NULL, ".\n");
      char *key_str = strtok(NULL, " \t\n");
      char *val_str = strtok(NULL, " \t\r\n");

      bool error = false;
      type = 0;
      sub = 0;
      key = 0;

      // Handle different commands
      if (strcmp(command, retreive_str(message_strs, MESSAGE_GET_KEY)) == 0) {
        type = MESSAGE_GET_KEY;
      } else if (strcmp(command, retreive_str(message_strs, MESSAGE_SET_KEY)) == 0) {
        type = MESSAGE_SET_KEY;
      } else if (strcmp(command, retreive_str(message_strs, MESSAGE_CAPTURE_KEY)) == 0) {
        type = MESSAGE_CAPTURE_KEY;
      } else if (strcmp(command, retreive_str(message_strs, MESSAGE_RESET_KEY)) == 0) {
        type = MESSAGE_RESET_KEY;
      } else {
        Serial.print(F("Unknown command: "));
        Serial.println(command);
        error = true;
      }

      // Set the sensor subsystem
      if (strcmp(sub_str, retreive_str(sensor_strs, SENSOR_PROSTATE)) == 0) {
        sub = SENSOR_PROSTATE;
      } else if (strcmp(sub_str, retreive_str(sensor_strs, SENSOR_ANUS)) == 0) {
        sub = SENSOR_ANUS;
      } else {
        Serial.print(F("Unknown sensor/subsystem: "));
        Serial.println(sub_str);
        error = true;
      }

      // Set the key
      if (strcmp(key_str, retreive_str(key_strs, KEY_STATUS)) == 0) {
        key = KEY_STATUS;

        // Interpret status names
        if (strcmp(val_str, retreive_str(status_strs, STATUS_OFF)) == 0)
          val = STATUS_OFF;
        else if (strcmp(val_str, retreive_str(status_strs, STATUS_ON)) == 0)
          val = STATUS_ON;
        else if (strcmp(val_str, retreive_str(status_strs, STATUS_LOG)) == 0)
          val = STATUS_LOG;
        
      } else {
        if (strcmp(key_str, retreive_str(key_strs, KEY_READING)) == 0) {
          key = KEY_READING;
        } else if (strcmp(key_str, retreive_str(key_strs, KEY_THRESHOLD_1)) == 0) {
          key = KEY_THRESHOLD_1;
        } else if (strcmp(key_str, retreive_str(key_strs, KEY_THRESHOLD_2)) == 0) {
          key = KEY_THRESHOLD_2;
        } else if (strcmp(key_str, retreive_str(key_strs, KEY_THRESHOLD_3)) == 0) {
          key = KEY_THRESHOLD_3;
        } else {
          Serial.print(F("Unknown key: "));
          Serial.println(key_str);
          error = true;
        }

        // Interpret key value
        if (strcmp(val_str, "inf") == 0)
          val = LONG_MAX;
        else
          val = atol(val_str);
      }

      // Construct CAN message
      message.id = (DEVICE_THIS << 7) | type;
      message.header.rtr = false;
      message.header.length = 2;
      message.data[0] = sub;
      message.data[1] = key;
      if (type == MESSAGE_SET_KEY) {
        memcpy(message.data + 2, &val, 4);
        message.header.length += 2;
      }

      if (!error) {
        // Send CAN message
        mcp2515_bit_modify(CANCTRL, (1 << REQOP2) | (1 << REQOP1) | (1 << REQOP0), 0);
        // ^ I don't know what this line does either
        mcp2515_send_message(&message);
      }

      // Command has been handled; indicate that the input buffer is 'cleared'
      input_len = 0;
    }
  }

  // Implement volume control using the CAN shield's onboard joystick
  uint16_t vol = 0;
  if (!digitalRead(PIN_JOY_DOWN)) {
    if (vol = sfx.volDown()) {
      Serial.print(F("Volume: "));
      Serial.println(vol);
    } /* else {
      digitalWrite(PIN_SFX_VOLDN, HIGH);
      delay(100);
      digitalWrite(PIN_SFX_VOLDN, LOW);
    } */
  } else if (!digitalRead(PIN_JOY_UP)) {
    if (vol = sfx.volUp()) {
      Serial.print(F("Volume: "));
      Serial.println(vol);
    } /* else {
      digitalWrite(PIN_SFX_VOLUP, HIGH);
      delay(100);
      digitalWrite(PIN_SFX_VOLUP, LOW);
    } */
  }
    
  // Update the SFX board every {SFX_UPDATE_RATE} milliseconds
  if (millis() - last_time >= SFX_UPDATE_RATE) {
    if (last_pain > 0) {
      current_track = (last_pain == 1) ? 2 : last_pain - 2;
      sfx_play_track(current_track);
      last_pain = 0;
    }

    last_time = millis();
  }
}

static void sfx_play_track(uint8_t track) {
  
  bool now_playing = !digitalRead(PIN_SFX_ACT);

//  // Check SFX board status
//  uint32_t remaining, total;
//  if (sfx.trackSize(&remaining, &total)) {
//    sfx_available = true;
//    now_playing = remaining > 0;
//  } else if (sfx_available) {    
//    Serial.println(F("Failed to get remaining track length"));
//    sfx_available = false;
//  }

  // Stop currently playing track if switching tracks
  if (now_playing && track != current_track) {
    if (sfx.stop()) {
      sfx_available = true;
    } else if (sfx_available) {
      Serial.println(F("Failed to stop currently playing track."));
    }
  }

  // Attempt to play track
  if (sfx.playTrack(track)) {
    sfx_available = true;
    current_track = track;
  } else if (sfx_available) {
    Serial.print(F("Failed to play track #"));
    Serial.print(track);
    Serial.println('.');
  }
}
