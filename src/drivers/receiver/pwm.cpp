#include "drivers/receiver/pwm.hpp"
#include "driver/rmt.h"
#include <array>

PWMReceiver::PWMReceiver() {
    type = RxType::PWM;
    init();
}

void PWMReceiver::init() {
    // Initialize the receiver

}

bool PWMReceiver::isConnected() {
    // Check if the receiver is connected
    return false;
}

void PWMReceiver::setChannel(uint8_t channel, uint16_t value) {
    // Set the value of a specific channel
}

void PWMReceiver::read() {
    // Read the receiver channels
}

constexpr std::array<uint8_t, 6> RECEIVER_GPIO = {35, 34, 39, 36, 27, 14};
constexpr int PWM_CENTER = 1500; // Center of receiver PWM pulse
constexpr int RX_DEADZONE = 2;   // Deadzone for receiver PWM pulse

constexpr int RMT_TICK_PER_US = 1;                                                 // determines how many clock cycles one "tick" is, [1..255] source is generally 80MHz APB clk
constexpr int RMT_RX_MAX_US = 3500;                                                // time before receiver goes idle (longer pulses will be ignored)
constexpr int PWM_CHANNELS_NUM = 6;                                                // Receiver PWM Channel Number
constexpr std::array<uint8_t, PWM_CHANNELS_NUM> PWM_CHANNELS = {1, 2, 3, 4, 5, 6}; // Change the channels according to the number of channels
constexpr int RMT_RX_CLK_DIV = (80000000 / RMT_TICK_PER_US / 1000000); // Divide 80Mhz APB clk by 1Mhz to get the clock for the receiver
/**
 * @brief Initialize RMT's interrupt handler
 * @details https://www.esp32.com/viewtopic.php?t=7116#p32383 (ESP32 Forum) this ISR only checks chX_rx_end
 * @param void
 * @return void
 */
static void rmt_isr_handler(void *arg)
{
  uint32_t intr_st = RMT.int_st.val;
  // declaration of RMT.int_st:
  // bit 0: ch0_tx_end
  // bit 1: ch0_rx_end
  // bit 2: ch0_err
  // bit 3: ch1_tx_end
  // bit 4: ch1_rx_end
  // check whether bit (channel*3 + 1) is set to identify if that channel has changed

  uint8_t i; // channel number
  for (i = 0; i < 6 /*pwm_ch_amt*/; i++)
  {                                                                // iterate through all channels
    uint8_t channel = CfgMan.getActiveCfg()->RECEIVER_CHANNELS[i]; // get channel number
    uint32_t channel_mask = BIT(channel * 3 + 1);                  // get channel mask

    if (!(intr_st & channel_mask))
      continue; // if channel has not changed, continue

    RMT.conf_ch[channel].conf1.rx_en = 0;                      // disable channel
    RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_TX;   // set channel to TX mode
    volatile rmt_item32_t *item = RMTMEM.chan[channel].data32; // get channel data pointer

    if (item)
    {
      FliCon.rx_raw[i] = item->duration0;
    } // if data is available, save it
    else
    {
      FliCon.rx_raw[i] = 0;
    } // if no data is available, set to 0

    RMT.conf_ch[channel].conf1.mem_wr_rst = 1;               // reset memory
    RMT.conf_ch[channel].conf1.mem_owner = RMT_MEM_OWNER_RX; // set channel to RX mode
    RMT.conf_ch[channel].conf1.rx_en = 1;                    // enable channel

    RMT.int_clr.val = channel_mask; // clear interrupt status
  }
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
/**
 * @brief Initialize RMT receiver
 * @details Initialize RMT receiver for PWM input
 * @return void
 * @note This function is called during setup()
 */
void rmt_init()
{
  uint8_t i; // channel number

  rmt_config_t rmt_channels[6 /*pwm_ch_amt*/] = {}; // initialize channel configuration array

  for (i = 0; i < 6 /*pwm_ch_amt*/; i++)
  {                                // iterate through all channels
    FliCon.rx_raw[i] = PWM_CENTER; // set raw data to PWM centerpoint

    rmt_channels[i].channel = (rmt_channel_t)CfgMan.getActiveCfg()->RECEIVER_CHANNELS[i]; // set channel number
    rmt_channels[i].gpio_num = (gpio_num_t)CfgMan.getActiveCfg()->RECEIVER_GPIOS[i];      // set GPIO number
    rmt_channels[i].clk_div = RMT_RX_CLK_DIV;                                             // set clock divider
    rmt_channels[i].mem_block_num = 1;                                                    // set memory block number
    rmt_channels[i].rmt_mode = RMT_MODE_RX;                                               // set mode to RX
    rmt_channels[i].rx_config.filter_en = true;                                           // enable filter
    rmt_channels[i].rx_config.filter_ticks_thresh = 100;                                  // set filter threshold
    rmt_channels[i].rx_config.idle_threshold = RMT_RX_MAX_US * RMT_TICK_PER_US;           // set idle threshold

    rmt_config(&rmt_channels[i]);                      // configure channel
    rmt_set_rx_intr_en(rmt_channels[i].channel, true); // enable interrupt
    rmt_rx_start(rmt_channels[i].channel, 1);          // start channel
  }

  rmt_isr_register(rmt_isr_handler, NULL, 0, NULL); // register interrupt handler function
}