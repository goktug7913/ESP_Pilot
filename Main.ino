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
#include "definitions.h"              //Hardcoded values and default configuration
#include "Config.h"
#include "ConfigSuite.h"              //Configuration manager, responsible for managing and verifying config
#include "PID.h"
#include "Controller.h"               //Master Flight Control Class
#include "Telemetry.h"
#include "SerialManager.h"
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// GLOBAL OBJECTS- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
FC FliCon;
FC_cfg cfg;
ConfigSuite CfgMan;
TelemetryManager Logger;
SerialMgr SerialMan;

SPIClass* hspi = nullptr; // SPI, instantiated in coldstart() during setup()
MPU6050 mpu(Wire);

//RF24 radio(RF24_CE, RF24_CSN, RF24_FREQ); // (CE,CSN,SPI CLK)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber = 1; // 0 uses address[0] to transmit, 1 uses address[1] to transmit
bool role = false;    // true = TX role, false = RX role
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
static void rmt_isr_handler(void* arg){
  // https://www.esp32.com/viewtopic.php?t=7116#p32383
  // this ISR only checks chX_rx_end

  uint32_t intr_st = RMT.int_st.val;
  // declaration of RMT.int_st:
  // bit 0: ch0_tx_end
  // bit 1: ch0_rx_end
  // bit 2: ch0_err
  // bit 3: ch1_tx_end
  // bit 4: ch1_rx_end
  // check whether bit (channel*3 + 1) is set to identify whether that channel has changed

  uint8_t i;
  for(i = 0; i < pwm_ch_amt; i++) {
    uint8_t channel = cfg.RECEIVER_CHANNELS[i];
    uint32_t channel_mask = BIT(channel*3+1);

    if (!(intr_st & channel_mask)) continue;

    RMT.conf_ch[channel].conf1.rx_en = 0;
    RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_TX;
    volatile rmt_item32_t* item = RMTMEM.chan[channel].data32;

    if (item) {FliCon.rx_raw[i] = item->duration0;}

    RMT.conf_ch[channel].conf1.mem_wr_rst = 1;
    RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_RX;
    RMT.conf_ch[channel].conf1.rx_en = 1;

    //clear RMT interrupt status
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
  // Program should not stay in this loop for more than a second or two during flight
  // You should get back into the flight loop as fast as possible if fallback happens

  //FliCon.writeEsc(1000,1000,1000,1000);
  SerialMan.ReceiveMsg();

  if(FliCon.rx_raw[4] == 2000){FliCon.armed = 1;}  // Arm on CH5 high

  //if(FliCon.usbmode){flightLoop_d();}

  if (FliCon.armed){FliCon.Start();} // Directly enter flight loop with debugging
}
// - - - - - - - - - - - - - - - - -
void coldstart(){
  Serial.begin(115200);
  while (!Serial) {}

  Wire.begin(-1, -1, 600000);
  
  EEPROM.begin(EEPROM_SIZE);
  CfgMan.loadCfg();
  CfgMan.applyDraftCfg();

  rmt_init();
  
  if (cfg.oled_display){
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
    for(;;); // Don't proceed, loop forever
  }*/

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.print(F(status));

  mpu.upsideDownMounting = MPU_UPSIDEDOWN;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Should detect if reboot happened mid flight at this point and recover offsets from EEPROM
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  mpu.calcOffsets(); // gyro and accelero
  delay(2000);
  Serial.println("Done!\n");

  radioNumber = 1 == 1;
  //radioSetup();
}
// - - - - - - - - - - - - - - - - -
void TempUpdate(){
  FliCon.temperature = mpu.getTemp();
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
void EmergencyAutoDescent(){}
// - - - - - - - - - - - - - - - - -
void TryHovering(){}
// - - - - - - - - - - - - - - - - -
void Fail(){}
// - - - - - - - - - - - - - - - - -
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