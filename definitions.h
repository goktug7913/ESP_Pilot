// DEFAULT DEFINES
#define SCREEN_WIDTH       128        //OLED display width, in pixels
#define SCREEN_HEIGHT      32         //OLED display height, in pixels

#define MPU_UPSIDEDOWN     1          //Enable if MPU chip faces downward!

#define PWM_FREQ           250        //ESC Control signal, Default: 50Hz, 100Hz for more responsiveness
#define PWM_MIN_DUTY       1000       //ESC Minimum Pulse Width
#define PWM_MAX_DUTY       2000       //ESC Maximum Pulse Width
#define ESC_1              26         //PWM0A Pin
#define ESC_2              25         //PWM0B Pin
#define ESC_3              33         //PWM1A Pin
#define ESC_4              32         //PWM1B Pin

#define RF24_CE            2          //nRF24 CE Pin
#define RF24_CSN           0          //nRF24 CSN Pin
#define RF24_FREQ          1000000    //nRF24 SPI Speed

#define CFG_MAGIC          0xDEADBEEF //Config Struct Header
#define CFG_MAGIC2         0xFEEBDAED //Config Struct Footer

#define EEPROM_START_ADDR  0          //EEPROM Data start address
#define EEPROM_SIZE        512        //EEPROM Allocated Size in Bytes (512 Max)
//GPIO 26 25 33 32 pwmout
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// ------------------- SYNC ANY CHANGES WITH ESP_CONFIGURATOR APP CODE ---------------------
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//Serial Commands

#define CFG_MODE          70    //Enable configurator
#define READ_CFG          72    //Read configuration from EEPROM
#define WRITE_CFG         73    //Write configuration to EEPROM
#define ARM_TETHERED      76    //Arm the quad while connected for debugging/testing
#define DISARM            77    //Disarm
#define RESTART_FC        78    //Reboot ESP32
#define TELEMETRY_START   91    //Request serial telemetry
#define TELEMETRY_STOP    92    //Stop serial telemetry

//ESP Callbacks
#define HANDSHAKE         71    //Serial connected
#define W_EEPROM_OK       74    //New configuration flashed succesfully
#define W_EEPROM_ERR      75    //New configuration flash failed

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//PWM RC Detection RMT Perhiperal Parameters

#define RMT_TICK_PER_US   1           //determines how many clock cycles one "tick" is, [1..255] source is generally 80MHz APB clk
#define RMT_RX_MAX_US     3500        //time before receiver goes idle (longer pulses will be ignored)
#define PWM_CHANNELS_NUM  5           //Receiver PWM Channel Number
#define PWM_CHANNELS      1,2,3,4,5//,6 // Change the channels according to the number of channels

#define RECEIVER_GPIO     35,34,39,36,27//,14
#define PWM_CENTER        1500
#define RX_DEADZONE       0
#define RMT_RX_CLK_DIV    (80000000/RMT_TICK_PER_US/1000000)
//ch1 yaw
//ch2 pitch
//ch3 throttle
//ch4 roll