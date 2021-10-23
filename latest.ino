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
#include <EEPROM.h>                   //Save-Load Configurations and mid-flight recovery
// - - - - - - - - - - - - - - - - -
#include "driver/mcpwm.h"             //ESC PWM control
#include "driver/rmt.h"               //Receiver PWM detection
// - - - - - - - - - - - - - - - - -
#include "definitions.h"              //Initialization values, default configuration
#include "Config.h"
#include "ConfigSuite.h"              //Configuration manager, responsible for managing and verifying config
#include "PID.h"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// GLOBAL OBJECTS- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//CfgMan, FliCon and cfg are created after defining them due to scope problems... (fix that)
FC FliCon;
PID PID_Ctrl;
FC_cfg cfg;
ConfigSuite CfgMan;
SPIClass* hspi = nullptr; // SPI, instantiated in coldstart() during setup()
RF24 radio(RF24_CE, RF24_CSN, RF24_FREQ); // (CE,CSN,SPI CLK)
MPU6050 mpu(Wire);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
bool role = false;    // true = TX role, false = RX role
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
static void rmt_isr_handler(void* arg){
    // with reference to https://www.esp32.com/viewtopic.php?t=7116#p32383
    // this ISR only checks chX_rx_end
    uint32_t intr_st = RMT.int_st.val;

    // see declaration of RMT.int_st:
    // takes the form of 
    // bit 0: ch0_tx_end
    // bit 1: ch0_rx_end
    // bit 2: ch0_err
    // bit 3: ch1_tx_end
    // bit 4: ch1_rx_end
    // ...
    // thus, check whether bit (channel*3 + 1) is set to identify
    // whether that channel has changed

    uint8_t i;
    for(i = 0; i < pwm_ch_amt; i++) {
        uint8_t channel = cfg.RECEIVER_CHANNELS[i];
        uint32_t channel_mask = BIT(channel*3+1);

        if (!(intr_st & channel_mask)) continue;

        RMT.conf_ch[channel].conf1.rx_en = 0;
        RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_TX;
        volatile rmt_item32_t* item = RMTMEM.chan[channel].data32;
        if (item) {
            FliCon.rx_raw[i] = item->duration0;
        }

        RMT.conf_ch[channel].conf1.mem_wr_rst = 1;
        RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_RX;
        RMT.conf_ch[channel].conf1.rx_en = 1;

        //clear RMT interrupt status.
        RMT.int_clr.val = channel_mask;
    }
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rmt_init() {
    uint8_t i;

    rmt_config_t rmt_channels[pwm_ch_amt] = {};

    for (i = 0; i < pwm_ch_amt; i++) {
        FliCon.rx_raw[i] = PWM_CENTER;

        rmt_channels[i].channel = (rmt_channel_t) cfg.RECEIVER_CHANNELS[i];
        rmt_channels[i].gpio_num = (gpio_num_t) cfg.RECEIVER_GPIOS[i];
        rmt_channels[i].clk_div = RMT_RX_CLK_DIV;
        rmt_channels[i].mem_block_num = 1;
        rmt_channels[i].rmt_mode = RMT_MODE_RX;
        rmt_channels[i].rx_config.filter_en = true;
        rmt_channels[i].rx_config.filter_ticks_thresh = 100;
        rmt_channels[i].rx_config.idle_threshold = RMT_RX_MAX_US * RMT_TICK_PER_US;

        rmt_config(&rmt_channels[i]);
        rmt_set_rx_intr_en(rmt_channels[i].channel, true);
        rmt_rx_start(rmt_channels[i].channel, 1);
    }

    rmt_isr_register(rmt_isr_handler, NULL, 0, NULL);
}
void setup(){
coldstart();
initEscDrive();
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
int counter;
bool ledset = 0;

void loop() {
  Receive();
  parseCommand();

  if(FliCon.rx_raw[4] == 2000){FliCon.armed = 1;}  // Arm on CH5 high
  
  if (FliCon.armed & !ledset){digitalWrite(2,HIGH); ledset = 1;} // Debug LED on if armed
  if (!FliCon.armed & ledset){digitalWrite(2,LOW); ledset = 0;}

  //if(FliCon.usbmode){flightLoop_d();}

  if (FliCon.armed){flightLoop_d();} // Directly enter flight loop with debugging
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
void flightLoop(){ // Primary Flight Loop (only flight essential functions)
  while(FliCon.armed){
    Receive();
    MotionUpdate();

    int esc1x,esc2x,esc3x,esc4x;
    esc1x = map( 3000 - FliCon.rx_raw[0] + 3000 - FliCon.rx_raw[1], 2000, 4000, 0, 8000 );
    esc2x = map(        FliCon.rx_raw[0] + 3000 - FliCon.rx_raw[1], 2000, 4000, 0, 8000 );
    esc3x = map( 3000 - FliCon.rx_raw[0] +        FliCon.rx_raw[1], 2000, 4000, 0, 8000 );
    esc4x = map(        FliCon.rx_raw[0] +        FliCon.rx_raw[1], 2000, 4000, 0, 8000 );

    writeEsc(esc1x,esc2x,esc3x,esc4x);
  }
}
// - - - - - - - - - - - - - - - - -
void flightLoop_d(){ // Flight Loop with serial enabled
  while(FliCon.armed){

    parseCommand();

    PID_Ctrl.Tstart = micros();

    MotionUpdate();
    PID_Ctrl.calculateTargets();

    writeEsc(PID_Ctrl.pid_esc1,
             PID_Ctrl.pid_esc2,
             PID_Ctrl.pid_esc3,
             PID_Ctrl.pid_esc4);

    if(FliCon.rx_raw[4] < 1250){FliCon.armed = 0;}  // Disarm if CH5 low
    
    PID_Ctrl.Tend = micros();
    Serial.println(PID_Ctrl.Tend-PID_Ctrl.Tstart);
  }
}
// - - - - - - - - - - - - - - - - -
void oledPrint(){
  display.clearDisplay();
  display.setCursor(0, 0);

  display.printf("rx: %d, %d, %d", FliCon.rx_raw[0],FliCon.rx_raw[1],FliCon.rx_raw[2]);
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

  Wire.begin(-1, -1, 600000);
  
  EEPROM.begin(EEPROM_SIZE);
  //CfgMan.loadCfg();
  //CfgMan.applyDraftCfg();

  rmt_init();
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.clearDisplay();

  //hspi = new SPIClass(HSPI);
  //hspi->begin();
  /* // initialize NRF24 on the SPI bus
  if (!radio.begin(hspi)) {
    Serial.println(F("radio hardware is not responding!!"));
    display.println(F("radio hardware is not responding!!"));
    for(;;); // Don't proceed, loop forever
  }*/

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.print(F(status));

  mpu.upsideDownMounting = 1;
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  mpu.calcOffsets(); // gyro and accelero
  delay(1500);
  Serial.println("Done!\n");

  radioNumber = 1 == 1;
  radioSetup();
}
// - - - - - - - - - - - - - - - - -
void Receive(){
  uint8_t pipe;
  if (radio.available(&pipe)) { // is there a payload? get the pipe number that recieved it
    uint8_t bytes = radio.getDynamicPayloadSize();
    radio.read(&FliCon.rx_raw, bytes); // fetch payload from FIFO
    radio.writeAckPayload(1,&FliCon,sizeof(FliCon));
  }
}
// - - - - - - - - - - - - - - - - -
void MotionUpdate(){
  mpu.update();
  FliCon.gyro[0]     = mpu.getAngleX();
  FliCon.gyro[1]     = mpu.getAngleY();
  FliCon.gyro[2]     = mpu.getAngleZ();

  FliCon.gyro_rps[0] = mpu.getGyroX();
  FliCon.gyro_rps[1] = mpu.getGyroY();
  FliCon.gyro_rps[2] = mpu.getGyroZ();

  FliCon.accel[0]    = mpu.getAccX();
  FliCon.accel[1]    = mpu.getAccY();
  FliCon.accel[2]    = mpu.getAccZ();
}
// - - - - - - - - - - - - - - - - -
void TempUpdate(){
  FliCon.temperature = mpu.getTemp();
}
// - - - - - - - - - - - - - - - - -
void radioSetup(){

  radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
  radio.setPayloadSize(sizeof(FliCon.rx_raw));

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
void initEscDrive(){
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, cfg.esc1_pin);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, cfg.esc2_pin);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, cfg.esc3_pin);
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, cfg.esc4_pin);

  mcpwm_config_t pwm_config = {
    .frequency = cfg.esc_pwm_hz,
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
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, esc1);
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, esc2);
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, esc3);
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, esc4);
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
  } else {return;}

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
      CfgMan.receiveCfg();
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
  writeEsc(0,0,0,0);
  FliCon.armed = 0;
  writeEsc(0,0,0,0);
}