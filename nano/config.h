// Baud rate for serial communications
#define BAUD_RATE 9600

// Defines how often the main loop should run [Hz]
#define REFRESH_RATE 200

// Default parameters
static const long defaults[NUM_SENSORS][NUM_KEYS] PROGMEM = {
  //  Status      Init. Value   Thresholds
  {   STATUS_ON,  0,            1000, 2000, 4000 },
  {   STATUS_ON,  0,              25,   50,  150 },
}
