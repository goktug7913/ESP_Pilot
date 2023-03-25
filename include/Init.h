// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
#include "freertos/FreeRTOS.h"        //ESP32 RTOS
#include "freertos/task.h"            //Task Scheduling
#include "esp_log.h"                  //ESP32 Logging
#include "string.h"                   //Memory operation
// - - - - - - - - - - - - - - - - -
//#include <SPI.h>                      //For nRF24 communication over SPI
//#include "RF24.h"                     //For nRF24 communication over SPI
// - - - - - - - - - - - - - - - - -
//#include <Wire.h>                     //I2C Bus for MPU6050, Magnetometer and OLED screen communication
//#include <MPU6050_light.h>            //For MPU6050
//#include <Adafruit_SSD1306.h>         //For OLED screen
//#include <Adafruit_GFX.h>             //For OLED screen
//#include <Adafruit_I2CDevice.h>       //For OLED screen
// - - - - - - - - - - - - - - - - -
//#include <EEPROM.h>                   //Save-Load Configurations and mid-flight recovery
// - - - - - - - - - - - - - - - - -
#include "driver/mcpwm.h"             //ESC PWM control
#include "driver/rmt.h"               //Receiver PWM detection
// - - - - - - - - - - - - - - - - -
#include "definitions.h"              //Hardcoded values and default configuration
#include "Config.h"                   //Configuration parameters
#include "ConfigSuite.h"              //Configuration manager, responsible for managing and verifying config

#include "Controller.h"               //Master Flight Control Class
#include "Telemetry.h"                //Telemetry class
#include "SerialManager.h"            //Serial Manager, responsible for managing serial communication
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// GLOBAL OBJECTS- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
ConfigSuite CfgMan;                   //Configuration manager, responsible for managing and verifying config
FC FliCon;                            //Flight Controller
FC_cfg cfg;                           //Flight Controller Configuration
TelemetryManager Logger;              //Telemetry manager, responsible for managing and sending telemetry
SerialMgr SerialMan;                  //Serial manager, responsible for managing and sending serial data

SPIClass* hspi = nullptr;             //SPI, instantiated in coldstart() during setup()
MPU6050 mpu(Wire);                    //MPU6050 Class

// Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RF24 radio(RF24_CE, RF24_CSN, RF24_FREQ); // (CE,CSN,SPI CLK)
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
bool role = false;    // true = TX role, false = RX role
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
static void rmt_isr_handler(void* arg){
  // RMT interrupt handler
  // https://www.esp32.com/viewtopic.php?t=7116#p32383
  // this ISR only checks chX_rx_end

  uint32_t intr_st = RMT.int_st.val;
  // declaration of RMT.int_st:
  // bit 0: ch0_tx_end
  // bit 1: ch0_rx_end
  // bit 2: ch0_err
  // bit 3: ch1_tx_end
  // bit 4: ch1_rx_end
  // check whether bit (channel*3 + 1) is set to identify if that channel has changed

  uint8_t i; // channel number
  for(i = 0; i < pwm_ch_amt; i++) { // iterate through all channels
    uint8_t channel = CfgMan.getActiveCfg()->RECEIVER_CHANNELS[i]; // get channel number
    uint32_t channel_mask = BIT(channel*3+1); // get channel mask

    if (!(intr_st & channel_mask)) continue; // if channel has not changed, continue

    RMT.conf_ch[channel].conf1.rx_en = 0; // disable channel
    RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_TX; // set channel to TX mode
    volatile rmt_item32_t* item = RMTMEM.chan[channel].data32; // get channel data pointer

    if (item) {FliCon.rx_raw[i] = item->duration0;} // if data is available, save it
    else {FliCon.rx_raw[i] = 0;} // if no data is available, set to 0

    RMT.conf_ch[channel].conf1.mem_wr_rst = 1; // reset memory
    RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_RX; // set channel to RX mode
    RMT.conf_ch[channel].conf1.rx_en = 1; // enable channel

    RMT.int_clr.val = channel_mask; // clear interrupt status
  }
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
void rmt_init() { // initialize RMT
    uint8_t i; // channel number

    rmt_config_t rmt_channels[pwm_ch_amt] = {}; // initialize channel configuration array

    for (i = 0; i < pwm_ch_amt; i++) { // iterate through all channels
        FliCon.rx_raw[i] = PWM_CENTER; // set raw data to PWM centerpoint

        rmt_channels[i].channel = (rmt_channel_t) CfgMan.getActiveCfg()->RECEIVER_CHANNELS[i]; // set channel number
        rmt_channels[i].gpio_num = (gpio_num_t) CfgMan.getActiveCfg()->RECEIVER_GPIOS[i]; // set GPIO number
        rmt_channels[i].clk_div = RMT_RX_CLK_DIV; // set clock divider
        rmt_channels[i].mem_block_num = 1; // set memory block number
        rmt_channels[i].rmt_mode = RMT_MODE_RX; // set mode to RX
        rmt_channels[i].rx_config.filter_en = true; // enable filter
        rmt_channels[i].rx_config.filter_ticks_thresh = 100; // set filter threshold
        rmt_channels[i].rx_config.idle_threshold = RMT_RX_MAX_US * RMT_TICK_PER_US; // set idle threshold

        rmt_config(&rmt_channels[i]); // configure channel
        rmt_set_rx_intr_en(rmt_channels[i].channel, true); // enable interrupt
        rmt_rx_start(rmt_channels[i].channel, 1); // start channel
    }

    rmt_isr_register(rmt_isr_handler, NULL, 0, NULL); // register interrupt handler function
}
// - - - - - - - - - - - - - - - - -
void coldstart(){
  //This function is called once at startup, and is responsible for initializing all hardware and objects
  //Also a reboot should be detected here, and recovery should be attempted if necessary

  Serial.begin(SERIAL_BAUD); // initialize serial port
  while (!Serial) {} //Wait for serial to be ready

  Wire.begin(-1, -1, 800000); //Start I2C, set speed to 800kHz
  
  EEPROM.begin(EEPROM_SIZE); 
  //CfgMan.getFlashCfg(); //Load config from flash

  rmt_init(); //Initialize RMT
  
  if (CfgMan.getActiveCfg()->oled_display){ //Initialize OLED if enabled
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
    }
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.clearDisplay();
  }

  //hspi = new SPIClass(HSPI);
  //hspi->begin();
  /* // initialize NRF24 on the SPI bus
  if (!radio.begin(hspi)) {
    Serial.println(F("radio hardware is not responding!!"));
    display.println(F("radio hardware is not responding!!"));
    for(;;); // Don't proceed if nRF24 fail
  }*/

  byte status = mpu.begin(); //Initialize MPU6050
  Serial.print(F("MPU6050 status: "));
  Serial.print(F(status));

  //mpu.upsideDownMounting = MPU_UPSIDEDOWN;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Should detect if reboot happened mid flight at this point and recover offsets from EEPROM
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  
  Serial.println(F("Calculating offsets, don't move the quad"));
  mpu.calcOffsets(); // gyro and accelero
  delay(1500); // wait for stable readings
  Serial.println("Done!\n");

  radioNumber = 1 == 1;
  //radioSetup();

  SerialMan.SendMsg(SERIALPOLL);
}
// - - - - - - - - - - - - - - - - -
void TempUpdate(){ //Update temperature
  FliCon.temperature = mpu.getTemp();
}
// - - - - - - - - - - - - - - - - -
void initEscDrive(){ //Initialize ESC Pins
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, CfgMan.getActiveCfg()->esc1_pin); //Initialize ESC1
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, CfgMan.getActiveCfg()->esc2_pin); //Initialize ESC2
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, CfgMan.getActiveCfg()->esc3_pin); //Initialize ESC3
  mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, CfgMan.getActiveCfg()->esc4_pin); //Initialize ESC4

  mcpwm_config_t pwm_config = { //Initialize PWM config
    .frequency = CfgMan.getActiveCfg()->esc_pwm_hz, //Set PWM frequency
    .cmpr_a = 0, // start duty cycle of PWMxA = 0
    .cmpr_b = 0, // start duty cycle of PWMxB = 0
    .duty_mode = MCPWM_DUTY_MODE_0, //Define duty mode
    .counter_mode = MCPWM_UP_COUNTER //Define counter mode
  };

  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Init ESC 1 & 2 Controller
  mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config); //Init ESC 3 & 4 Controller
}
// - - - - - - - - - - - - - - - - -
/*void Receive(){
  uint8_t pipe;
  if (radio.available(&pipe)) { // is there a payload? get the pipe number that recieved it
    uint8_t bytes = radio.getDynamicPayloadSize();
    radio.read(&FliCon.rx_raw, bytes); // fetch payload from FIFO
    radio.writeAckPayload(1,&FliCon,sizeof(FliCon));
  }
}*/
// - - - - - - - - - - - - - - - - -
/*void radioSetup(){

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
}*/