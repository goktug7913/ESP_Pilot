#ifndef DEFINITIONS_H
#define DEFINITIONS_H

// DEFAULT DEFINES
#define SCREEN_WIDTH       128        //OLED display width, in pixels
#define SCREEN_HEIGHT      32         //OLED display height, in pixels

// MPU
#define MPU_UPSIDEDOWN     1          //Enable if MPU chip faces downward (default is facing up)

// Pins and ESC
#define PWM_FREQ           250        //ESC Control signal, Default: 50Hz
#define PWM_MIN_DUTY       1000       //ESC Minimum Pulse Width (us)
#define PWM_MAX_DUTY       2000       //ESC Maximum Pulse Width (us)
#define ESC_1              26         //PWM0A Pin
#define ESC_2              25         //PWM0B Pin
#define ESC_3              33         //PWM1A Pin
#define ESC_4              32         //PWM1B Pin

// Control
#define ITERMSCALAR        1         //Integral term scalar (TESTING)
#define ITERMLIMIT         1280       //Integral term limit (TESTING)
#define ITERMDEADBAND      0.5        //Integral term deadband degrees (TESTING)
#define PIDLIMIT           475        //PID PWM output limit (TESTING)
#define PIDMASTERGAIN      1.0        //PID master gain (TESTING)

// Radio
#define RF24_CE            2          //nRF24 CE Pin
#define RF24_CSN           0          //nRF24 CSN Pin
#define RF24_FREQ          1000000    //nRF24 SPI Speed

// No need to change these, unless known what you are doing
#define CFG_MAGIC          0xDEADBEEF //Config Struct Header (32 bits)
#define CFG_MAGIC2         0xFEEBDAED //Config Struct Footer (32 bits)
#define TMTY_HEADER        0x533D     //Telemetry Frame Header (16 bits)
#define MSG_START          0xB1       //Message Start Flag (8 bits)
#define DATA_START         0xD2       //Data Start Flag (8 bits)
#define MSG_END            0xB2       //Message End Flag (8 bits)
#define DATA_END           0xD1       //Data End Flag (8 bits)

#define EEPROM_START_ADDR  0          //EEPROM Data start address
#define EEPROM_SIZE        512        //EEPROM Allocated Size in Bytes (512 bytes max)
#define SERIAL_BAUD        115200     //Serial Baud Rate
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// ------------------- SYNC ANY CHANGES WITH ESP_CONFIGURATOR APP CODE ---------------------
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Changing these values will break the configurator app. Only change if you know what you are doing.
// We will depricate these in the future as we move to AP mode and a web interface.
// Serial Commands
#define CFG_MODE          50    //Enable configurator
#define READ_CFG          51    //Read configuration from EEPROM
#define WRITE_CFG         52    //Write configuration to EEPROM
#define ARM_TETHERED      53    //Arm the quad while connected for debugging/testing
#define DISARM            54    //Disarm
#define RESTART_FC        55    //Reboot ESP32
#define TELEMETRY_START   56    //Request serial telemetry
#define TELEMETRY_STOP    57    //Stop serial telemetry
#define SET_PID_GAIN      90    //Set PID gain

// ESP Callbacks
#define HANDSHAKE         58    //Serial connected
#define SERIALPOLL        48    //ESP32 Polling for serial connection
#define CFG_DATA_FLAG     59    //Indicate data is outbound
#define TMTY_DATA_FLAG    60    //Indicate telemetry frame is outbound
#define W_EEPROM_OK       61    //New configuration flashed succesfully
#define W_EEPROM_ERR      62    //New configuration flash failed
#define S_EEPROM_ERR      63    //New configuration data is invalid
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//PWM RC Detection RMT Perhiperal Parameters

// We will use a fixed array of 6 for axis data for now
// TODO: Add support for arbitrary number of axes and digital inputs

#define RECEIVER_GPIO     35,34,39,36,27,14
#define PWM_CENTER        1500          //Center of receiver PWM pulse
#define RX_DEADZONE       2             //Deadzone for receiver PWM pulse

#define RMT_TICK_PER_US   1             //determines how many clock cycles one "tick" is, [1..255] source is generally 80MHz APB clk
#define RMT_RX_MAX_US     3500          //time before receiver goes idle (longer pulses will be ignored)
#define PWM_CHANNELS_NUM  6             //Receiver PWM Channel Number
#define PWM_CHANNELS      1,2,3,4,5,6   // Change the channels according to the number of channels

#define RMT_RX_CLK_DIV    (80000000/RMT_TICK_PER_US/1000000) //Divide 80Mhz APB clk by 1Mhz to get the clock for the receiver

/*
    These are my own channel assignments:
    ch1 yaw
    ch2 pitch
    ch3 throttle
    ch4 roll
    ch5 arm switch 3 way
*/

#endif // DEFINITIONS_H