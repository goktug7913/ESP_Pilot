// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
#include "freertos/FreeRTOS.h"        //ESP32 RTOS
#include "freertos/task.h"            //Task Scheduling
#include "esp_log.h"
#include "string.h"                   //Memory operation
// - - - - - - - - - - - - - - - - -
#include <SPI.h>                      //For nRF24 communication over SPI
#include "RF24.h"                     //For nRF24 communication over SPI
// - - - - - - - - - - - - - - - - -
#include <Wire.h>                     //I2C Bus for MPU6050, Magnetometer and OLED screen communication
#include <MPU6050_light.h>            //For MPU6050
#include <Adafruit_SSD1306.h>         //For OLED screen
#include <Adafruit_GFX.h>             //For OLED screen
// - - - - - - - - - - - - - - - - -
#include <array>                      //Better arrays
#include <EEPROM.h>                   //Save-Load Configurations and mid-flight recovery
// - - - - - - - - - - - - - - - - -
#include "driver/mcpwm.h"             //ESC PWM control
#include "driver/rmt.h"               //Receiver PWM detection
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
// DEFAULT DEFINES
#define SCREEN_WIDTH       128        //OLED display width, in pixels
#define SCREEN_HEIGHT      32         //OLED display height, in pixels

#define PWM_FREQ           100        //ESC Control signal, Default: 50Hz, 100Hz for more responsiveness
#define PWM_MIN_DUTY       1000       //Minimum Pulse Width
#define PWM_MAX_DUTY       2000       //Maximum Pulse Width
#define ESC_1              26         //PWM0A Pin
#define ESC_2              25         //PWM0B Pin
#define ESC_3              33         //PWM1A Pin
#define ESC_4              32         //PWM1B Pin

#define RF24_CE            2          //nRF24 CE Pin
#define RF24_CSN           0          //nRF24 CSN Pin
#define RF24_FREQ          1000000    //nRF24 SPI Speed

#define CFG_MAGIC          0xCF        //Config Struct Header
#define EEPROM_START_ADDR  0          //EEPROM Data start address
#define EEPROM_SIZE        512        //EEPROM Allocated Size in Bytes (512 Max)

#define PWM_CHANNELS_NUM   4          //Receiver PWM Channel Number
//GPIO 26 25 33 32 pwmout
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// ------------------- SYNC ANY CHANGES WITH ESP_CONFIGURATOR APP CODE ---------------------
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Two Byte Codes for reporting over serial

#define CFG_MODE        70  //Enable configurator if c1 is received
#define HANDSHAKE       71  //Serial connected
#define READ_CFG        72  //Read configuration from EEPROM
#define WRITE_CFG       73  //Write configuration to EEPROM
#define W_EEPROM_OK     74  //New configuration flashed succesfully
#define W_EEPROM_ERR    75  //New configuration flash failed
#define ARM_TETHERED    76  //Arm the quad while connected for debugging/testing
#define DISARM          77  //Disarm
#define RESTART_FC      78  //Reboot ESP32

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//PWM Detection with RMT perhiperal WIP
#define RMT_TICK_PER_US 1 // determines how many clock cycles one "tick" is, [1..255] source is generally 80MHz APB clk
#define RMT_RX_CLK_DIV (80000000/RMT_TICK_PER_US/1000000)
#define RMT_RX_MAX_US 3500 // time before receiver goes idle (longer pulses will be ignored)

volatile uint16_t pwmBuf[PWM_CHANNELS_NUM] = {0};
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// ------------SYNC ANY CHANGES TO THIS STRUCT WITH ESP_CONFIGURATOR APP CODE --------------
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
typedef struct {
  uint8_t header = CFG_MAGIC;

  //Control
  float   p_gain, i_gain, d_gain = 1.00;
  uint8_t max_angle = 20;

  //Settings
  float   vBat = 11.1;
  bool    vBat_compensation = 0;

  //Aux modules
  bool    nrf24_telemetry = 1;
  bool    oled_display = 1;
  bool    radar_altimeter = 0;
  bool    compass = 0;

  //ESC signal pins (initialized as defined in code)
  uint8_t esc1_pin = ESC_1;
  uint8_t esc2_pin = ESC_2;
  uint8_t esc3_pin = ESC_3;
  uint8_t esc4_pin = ESC_4;
} FC_cfg;
static FC_cfg cfg;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
typedef struct{
  std::array<float, 3> gyro;
  std::array<float, 3> gyro_rps;
  std::array<float, 3> accel;
  std::array<int, 3> rx_values;

  float temperature;
  bool armed = 0;
  bool recovery = 0;
  bool usbmode = 0;
} FC;
static FC FliCon;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
class ConfigSuite{
  public:
  int cfgsize = sizeof(current_config);
  int eeprom_size = EEPROM_SIZE;

  void sendCfg(){
    Serial.write((byte*)&current_config, sizeof(current_config)); //weird casting here lol
  }

  void applyDraftCfg(){memcpy(&cfg, &current_config, cfgsize);}

  void loadCfg(){
    FC_cfg candidate_cfg; // Temporary config struct object for checking the magic header
    byte data[cfgsize];   // Buffer containing the bytes read from the EEPROM

    for (int i = EEPROM_START_ADDR; i < cfgsize; i++){data[i] = EEPROM.read(i);}

    memcpy(&candidate_cfg, &data, cfgsize); //Load the candidate configuration to draft configuration
    if (candidate_cfg.header == CFG_MAGIC){memcpy(&current_config, &candidate_cfg, cfgsize);}
    else{Serial.write(W_EEPROM_ERR);}
  }

  bool writeCfg(){
    byte w_data[cfgsize];   // Buffer containing the bytes to write to the EEPROM
    for (int i = 0; i < cfgsize; i++){w_data[i] = Serial.read();}
    
    memcpy(&current_config, &w_data, cfgsize); //Load the received config data

    if(current_config.header == CFG_MAGIC){
      for (int i = EEPROM_START_ADDR; i < cfgsize; i++){EEPROM.write(i, w_data[i]);}
      if(!EEPROM.commit()){Serial.write(W_EEPROM_ERR);}
    }

    // CHECK IF FLASHED CORRECTLY
    FC_cfg candidate_cfg;   // Candidate Config data read back
    byte r_data[cfgsize];   // Buffer containing the bytes read from the EEPROM

    for (int i = EEPROM_START_ADDR; i < cfgsize; i++){r_data[i] = EEPROM.read(i);}

    memcpy(&candidate_cfg, &r_data, cfgsize); //Load back the candidate configuration

    if (memcmp(&candidate_cfg, &current_config, cfgsize) == 0){
      applyDraftCfg();
      Serial.write(W_EEPROM_OK);
      return 1;
    } else{
      Serial.write(W_EEPROM_ERR);
      return 0;
    }
  }

  private:
  FC_cfg current_config;
};
static ConfigSuite CfgMan;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// GLOBAL OBJECTS- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//CfgMan, FliCon and cfg are created after defining them due to scope problems... (fix that)

SPIClass* hspi = nullptr; // SPI, instantiated in coldstart() during setup()

RF24 radio(RF24_CE, RF24_CSN, RF24_FREQ); // (CE,CSN,SPI CLK)
MPU6050 mpu(Wire);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
bool role = false;  // true = TX role, false = RX role
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void setup(){
coldstart();
initEscDrive();
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
void loop() {

  Receive();
  parseCommand();

  if(FliCon.rx_values[2] == 1){FliCon.armed = 1;}

  if(FliCon.usbmode){flightLoop_d();}

  flightLoop();

  oledPrint();
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void flightLoop(){
  while(FliCon.armed){
    Receive();
    MotionUpdate();

    int esc1x,esc2x,esc3x,esc4x;
    esc1x = map( 3000 - FliCon.rx_values[0] + 3000 - FliCon.rx_values[1], 2000, 4000, 0, 8000 );
    esc2x = map(        FliCon.rx_values[0] + 3000 - FliCon.rx_values[1], 2000, 4000, 0, 8000 );
    esc3x = map( 3000 - FliCon.rx_values[0] +        FliCon.rx_values[1], 2000, 4000, 0, 8000 );
    esc4x = map(        FliCon.rx_values[0] +        FliCon.rx_values[1], 2000, 4000, 0, 8000 );

    writeEsc(esc1x,esc2x,esc3x,esc4x);
    oledPrint();
  }
}
// - - - - - - - - - - - - - - - - -
void flightLoop_d(){
  while(FliCon.armed){

    parseCommand();

    Receive();
    MotionUpdate();

    int esc1x,esc2x,esc3x,esc4x;
    esc1x = map( 3000 - FliCon.rx_values[0] + 3000 - FliCon.rx_values[1], 2000, 4000, 0, 8000 );
    esc2x = map(        FliCon.rx_values[0] + 3000 - FliCon.rx_values[1], 2000, 4000, 0, 8000 );
    esc3x = map( 3000 - FliCon.rx_values[0] +        FliCon.rx_values[1], 2000, 4000, 0, 8000 );
    esc4x = map(        FliCon.rx_values[0] +        FliCon.rx_values[1], 2000, 4000, 0, 8000 );

    writeEsc(esc1x,esc2x,esc3x,esc4x);
    oledPrint();
  }
}
// - - - - - - - - - - - - - - - - -
void oledPrint(){
  display.clearDisplay();
  display.setCursor(0, 0);

  display.printf("rx: %d, %d, %d", FliCon.rx_values[0],FliCon.rx_values[1],FliCon.rx_values[2]);
  display.print("\n");

  display.printf("dX: %.2f, aX: %.2f",FliCon.gyro[0], FliCon.accel[0]);
  display.print("\n");

  display.printf("dY: %.2f, aY: %.2f",FliCon.gyro[1], FliCon.accel[1]);
  display.print("\n");

  display.printf("dZ: %.2f, aZ: %.2f",FliCon.gyro[2], FliCon.accel[2]);

  display.display();
}
// - - - - - - - - - - - - - - - - -
void coldstart(){
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin(-1, -1, 400000);
  
  EEPROM.begin(EEPROM_SIZE);
  CfgMan.loadCfg();
  CfgMan.applyDraftCfg();
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.clearDisplay();

  hspi = new SPIClass(HSPI);
  hspi->begin();

  // initialize NRF24 on the SPI bus
  if (!radio.begin(hspi)) {
    Serial.println(F("radio hardware is not responding!!"));
    display.println(F("radio hardware is not responding!!"));
    for(;;); // Don't proceed, loop forever
  }

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  display.println("MPU6050 status: ");
  display.print(status);
  display.display();
  
  mpu.upsideDownMounting = 1;
  display.println(F("Calculating offsets, do not move MPU6050"));
  display.display();
  mpu.calcOffsets(); // gyro and accelero
  delay(1500);
  Serial.println("Done!\n");
  display.println("Done!\n");
  display.display(); 

  radioNumber = 1 == 1;
  radioSetup();

  radio.printPrettyDetails();

  display.display();
}
// - - - - - - - - - - - - - - - - -
void Receive(){
  uint8_t pipe;
  if (radio.available(&pipe)) { // is there a payload? get the pipe number that recieved it
    uint8_t bytes = radio.getDynamicPayloadSize();
    radio.read(&FliCon.rx_values, bytes); // fetch payload from FIFO
    radio.writeAckPayload(1,&FliCon,sizeof(FliCon));
  } else {radioFailcheck();}
}
// - - - - - - - - - - - - - - - - -
void MotionUpdate(){
  mpu.update();
  FliCon.gyro[0] = mpu.getAngleX();
  FliCon.gyro[1] = mpu.getAngleY();
  FliCon.gyro[2] = mpu.getAngleZ();

  FliCon.accel[0] = mpu.getAccX();
  FliCon.accel[1] = mpu.getAccY();
  FliCon.accel[2] = mpu.getAccZ();
}
// - - - - - - - - - - - - - - - - -
void TempUpdate(){
  FliCon.temperature = mpu.getTemp();
}
// - - - - - - - - - - - - - - - - -
void radioSetup(){

  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  radio.setPayloadSize(sizeof(FliCon.rx_values));

  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.writeAckPayload(1,&FliCon,sizeof(FliCon));

  // set the TX address of the RX node into the TX pipe
  radio.openWritingPipe(address[radioNumber]);     // always uses pipe 0

  // set the RX address of the TX node into a RX pipe
  radio.openReadingPipe(1, address[!radioNumber]); // using pipe 1

  radio.startListening(); // put radio in RX mode
}
// - - - - - - - - - - - - - - - - -
void radioFailcheck(){ // Will be deprecated after connecting fs-ia6
  if(radio.failureDetected){
    radioSetup();
    radio.failureDetected = 0;
    Serial.println("Radio failure: Trying to reset nRF24...");
    //in progress
  }

  if(!radio.isChipConnected()){
    Serial.println("NRF24 SPI CONNECTION LOST!");
    radioSetup(); //Does not reset nrf24 ? 
  }
}
// - - - - - - - - - - - - - - - - -
void initEscDrive(){
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, cfg.esc1_pin);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, cfg.esc2_pin);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, cfg.esc3_pin);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, cfg.esc4_pin);

  mcpwm_config_t pwm_config = {
    .frequency = PWM_FREQ,
    .cmpr_a = 0, // start duty cycle of PWMxA = 0
    .cmpr_b = 0, // start duty cycle of PWMxB = 0
    .duty_mode = MCPWM_DUTY_MODE_0,
    .counter_mode = MCPWM_UP_COUNTER
  };

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Init ESC 1 & 2 Controller
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config); //Init ESC 3 & 4 Controller
}
// - - - - - - - - - - - - - - - - -
void writeEsc(uint32_t esc1, uint32_t esc2, uint32_t esc3, uint32_t esc4){
  ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, esc1));
  ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, esc2));
  ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, esc3));
  ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, esc4));
}
// - - - - - - - - - - - - - - - - -
void EmergencyAutoDescent(){}
// - - - - - - - - - - - - - - - - -
void TryHovering(){}
// - - - - - - - - - - - - - - - - -
void Fail(){}
// - - - - - - - - - - - - - - - - -
void parseCommand(){
  uint8_t cmd;

  if (Serial.available() > 0) {
    // wait for cfg mode command
    cmd = Serial.read();
  }

  switch (cmd){
    // - - - - - - - - - - - - - - - - -
    case CFG_MODE:
      FliCon.usbmode = 1;
      Serial.write(HANDSHAKE);
    break;
    // - - - - - - - - - - - - - - - - -
    case READ_CFG:
      CfgMan.loadCfg();
      CfgMan.sendCfg();
    break;
    // - - - - - - - - - - - - - - - - -
    case WRITE_CFG:
      CfgMan.writeCfg();
    break;
    // - - - - - - - - - - - - - - - - -
    case ARM_TETHERED:
      FliCon.armed = 1;
    break;
    // - - - - - - - - - - - - - - - - -
    case DISARM:
      disarm();
    break;
    // - - - - - - - - - - - - - - - - -
    case RESTART_FC:
      ESP.restart();
    break;
    // - - - - - - - - - - - - - - - - -
  }
}
// - - - - - - - - - - - - - - - - -
void disarm(){
  writeEsc(0,0,0,0);
  writeEsc(0,0,0,0);
  FliCon.armed = 0;
  writeEsc(0,0,0,0);
  writeEsc(0,0,0,0);
}