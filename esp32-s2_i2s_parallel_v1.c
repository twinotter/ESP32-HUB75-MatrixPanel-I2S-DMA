/*
 * ESP32-S2_I2S_PARALLEL_V1 (Version 1)
 * 
 * Author:      Mrfaptastic - https://github.com/mrfaptastic/
 * 
 * Description: Implementation of ESP32-S2 DMA setup for ESP32-S2 SoC, which actually has
 *				half decent technical documentation to make this possible.
 *
 */
 
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <driver/gpio.h>
#include <driver/periph_ctrl.h>
#include <rom/gpio.h>
#include <soc/gpio_sig_map.h>

// Header
#include <esp32-s2_i2s_parallel_v1.h>

// For I2S<N> frame buffer state management.
static i2s_parallel_state_t *i2s_state = NULL;

callback shiftCompleteCallback;
void setShiftCompleteCallback(callback f) {
    shiftCompleteCallback = f;
}

volatile bool previousBufferFree = true;

static void IRAM_ATTR irq_hndlr(void* arg) {

//    if ( (*(i2s_port_t*)arg) == I2S_NUM_0 ) { // https://www.bogotobogo.com/cplusplus/pointers2_voidpointers_arrays.php
      SET_PERI_REG_BITS(I2S_INT_CLR_REG(0), I2S_OUT_EOF_INT_CLR_V, 1, I2S_OUT_EOF_INT_CLR_S);    
//    }

    // at this point, the previously active buffer is free, go ahead and write to it
    previousBufferFree = true;

    if(shiftCompleteCallback) // we've defined a callback function ?
        shiftCompleteCallback();
}


// For peripheral setup and configuration
static i2s_dev_t* I2S[I2S_NUM_MAX] = {&I2S0};

static inline int get_bus_width(i2s_parallel_cfg_bits_t width) {
  switch(width) {
    case I2S_PARALLEL_WIDTH_8:
      return 8;
    case I2S_PARALLEL_WIDTH_16:
      return 16;
    case I2S_PARALLEL_WIDTH_24:
      return 24;
    default:
      return -ESP_ERR_INVALID_ARG;
  }
}

static void iomux_set_signal(int gpio, int signal) {
  if(gpio < 0) {
    return;
  }
  PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[gpio], PIN_FUNC_GPIO);
  gpio_set_direction(gpio, GPIO_MODE_DEF_OUTPUT);
  gpio_matrix_out(gpio, signal, false, false);
}

static void dma_reset(i2s_dev_t* dev) {
  dev->lc_conf.in_rst = 1;
  dev->lc_conf.in_rst = 0;
  dev->lc_conf.out_rst = 1;
  dev->lc_conf.out_rst = 0;
  
  dev->lc_conf.ahbm_rst = 1;
  dev->lc_conf.ahbm_rst = 0;
    
  
}

static void fifo_reset(i2s_dev_t* dev) {
  dev->conf.rx_fifo_reset = 1;
  while(dev->conf.rx_fifo_reset_st);
  dev->conf.rx_fifo_reset = 0;
  
  dev->conf.tx_fifo_reset = 1;
  while(dev->conf.tx_fifo_reset_st);
  dev->conf.tx_fifo_reset = 0;
  
}

static void dev_reset(i2s_dev_t* dev) {
  fifo_reset(dev);
  dma_reset(dev);
  dev->conf.rx_reset=1;
  dev->conf.tx_reset=1;
  dev->conf.rx_reset=0;
  dev->conf.tx_reset=0;  
}

// DMA Linked List
// Size must be less than DMA_MAX - need to handle breaking long transfer into two descriptors before call
// DMA_MAX by the way is the maximum data packet size you can hold in one chunk
void link_dma_desc(volatile lldesc_t *dmadesc, volatile lldesc_t *prevdmadesc, void *memory, size_t size) 
{
    if(size > DMA_MAX) size = DMA_MAX;

    dmadesc->size = size;
    dmadesc->length = size;
    dmadesc->buf = memory;
    dmadesc->eof = 0;
    dmadesc->sosf = 0;
    dmadesc->owner = 1;
    dmadesc->qe.stqe_next = 0;  // will need to set this elsewhere
    dmadesc->offset = 0;

    // link previous to current
    if(prevdmadesc)
        prevdmadesc->qe.stqe_next = (lldesc_t*)dmadesc;
}


// ESP32-S2 only has IS20
esp_err_t i2s_parallel_driver_install(i2s_port_t port, i2s_parallel_config_t* conf) {
  if(port < I2S_NUM_0 || port >= I2S_NUM_MAX) {
    return ESP_ERR_INVALID_ARG;
  }
  if(conf->sample_width < I2S_PARALLEL_WIDTH_8 || conf->sample_width >= I2S_PARALLEL_WIDTH_MAX) {
    return ESP_ERR_INVALID_ARG;
  }
  if(conf->sample_rate > I2S_PARALLEL_CLOCK_HZ || conf->sample_rate < 1) {
    return ESP_ERR_INVALID_ARG;
  }
  
  uint32_t clk_div_main = I2S_PARALLEL_CLOCK_HZ / conf->sample_rate / i2s_parallel_get_memory_width(port, conf->sample_width);
  if(clk_div_main < 2 || clk_div_main > 0xFF) {
    return ESP_ERR_INVALID_ARG;
  }

  volatile int iomux_signal_base;
  volatile int iomux_clock;
  int irq_source;
  
  // Initialize I2S0 peripheral
  periph_module_reset(PERIPH_I2S0_MODULE);
  periph_module_enable(PERIPH_I2S0_MODULE);
  iomux_clock = I2S0O_WS_OUT_IDX;
  irq_source = ETS_I2S0_INTR_SOURCE;

  switch(conf->sample_width) {
	case I2S_PARALLEL_WIDTH_8:
	case I2S_PARALLEL_WIDTH_16:
	  iomux_signal_base = I2S0O_DATA_OUT8_IDX;
	  break;
	case I2S_PARALLEL_WIDTH_24:
	  iomux_signal_base = I2S0O_DATA_OUT0_IDX;
	  break;
	case I2S_PARALLEL_WIDTH_MAX:
	  return ESP_ERR_INVALID_ARG;
  }


  // Setup GPIOs
  int bus_width = get_bus_width(conf->sample_width);

  // Setup I2S peripheral
  i2s_dev_t* dev = I2S[port];
  
  // Setup GPIO's
  for(int i = 0; i < bus_width; i++) {
    iomux_set_signal(conf->gpio_bus[i], iomux_signal_base + i);
  }
  iomux_set_signal(conf->gpio_clk, iomux_clock);


  // invert clock phase if required
  if (conf->clkphase)
    GPIO.func_out_sel_cfg[conf->gpio_clk].inv_sel = 1;  
  
  // Setup i2s clock
  dev->sample_rate_conf.val = 0;
  dev->sample_rate_conf.rx_bits_mod = bus_width;
  dev->sample_rate_conf.tx_bits_mod = bus_width;
  
  dev->sample_rate_conf.rx_bck_div_num = 2;
  dev->sample_rate_conf.tx_bck_div_num = 2; // datasheet says this must be 2 or greater (but 1 seems to work)

  dev->clkm_conf.val=0;
  dev->clkm_conf.clk_sel = 2;
  dev->clkm_conf.clk_en	 = 1;
  dev->clkm_conf.clkm_div_a=0;
  dev->clkm_conf.clkm_div_b=1;   
  
  // Note: clkm_div_num must only be set here AFTER clkm_div_b, clkm_div_a, etc. Or weird things happen!
  dev->clkm_conf.clkm_div_num = clk_div_main;

  // Set i2s mode to LCD mode
  dev->conf2.val = 0;
  dev->conf2.lcd_en = 1;
  dev->conf2.lcd_tx_wrx2_en=0; 
  dev->conf2.lcd_tx_sdx2_en=0; 
  
  dev->conf.val = 0;  
  //dev->conf.tx_slave_mod = 0; 
  
  dev->conf.tx_dma_equal=1; // esp32-s2 only
  //dev->conf.tx_mono=0; // esp32-s2 only    
  //dev->conf_chan.tx_chan_mod = 0;
  //dev->conf.tx_right_first=0; // esp32-s2 only
  //dev->conf.tx_big_endian=0; // esp32-s2 only  
  //dev->conf.tx_short_sync=0; // esp32-s2 only  
  //dev->conf.tx_msb_shift=0; // esp32-s2 only  
  //dev->conf.tx_msb_right=0; // esp32-s2 only    
  dev->conf.pre_req_en=1; // enable I2S to prepare data earlier? wtf?
  

  // Tx in mono mode, read 32 bit per sample from fifo
  

  // Enable DMA support
  dev->fifo_conf.rx_data_num = 32; // Thresholds. 
  dev->fifo_conf.tx_data_num = 32;
  
  dev->fifo_conf.dscr_en = 1;
  
  // Reset
  dev_reset(dev);  
  dev->conf1.val = 0;
  dev->conf1.tx_stop_en = 0;
  
  for(int i = 0; i < 10000000; i++ ) { }
  
  // Allocate I2S status structure for buffer swapping stuff
  i2s_state = (i2s_parallel_state_t*) malloc(sizeof(i2s_parallel_state_t));
  assert(i2s_state != NULL);
  i2s_parallel_state_t *state = i2s_state;
  
  state->desccount_a    = conf->desccount_a;
  state->desccount_b    = conf->desccount_b;
  state->dmadesc_a      = conf->lldesc_a;
  state->dmadesc_b      = conf->lldesc_b;  
  state->i2s_interrupt_port_arg  = port; // need to keep this somewhere in static memory for the ISR

  // Get ISR setup 
  esp_err_t err =  esp_intr_alloc(irq_source, 
                                 (int)(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1),
                                 irq_hndlr, 
                                 &state->i2s_interrupt_port_arg, NULL);

  if(err) {
      return err;
  }  

  dev->timing.val = 0;

  // Setup interrupt handler which is focussed only on the (page 322 of Tech. Ref. Manual)
  // "I2S_OUT_EOF_INT: Triggered when rxlink has finished sending a packet"
  // ... whatever the hell that is supposed to mean... One massive linked list.
  dev->int_ena.out_eof = 1;


  return ESP_OK;
}

 esp_err_t i2s_parallel_stop_dma(i2s_port_t port) {
  if(port < I2S_NUM_0 || port >= I2S_NUM_MAX) {
    return ESP_ERR_INVALID_ARG;
  }

  i2s_dev_t* dev = I2S[port];
  
  // Stop all ongoing DMA operations
  dev->out_link.stop = 1;
  dev->out_link.start = 0;
  dev->conf.tx_start = 0;
  
   return ESP_OK;
}


 esp_err_t i2s_parallel_send_dma(i2s_port_t port, lldesc_t* dma_descriptor) {
  if(port < I2S_NUM_0 || port >= I2S_NUM_MAX) {
    return ESP_ERR_INVALID_ARG;
  }

  i2s_dev_t* dev = I2S[port];
  // Stop all ongoing DMA operations
  //dev->out_link.stop = 1;
  //dev->out_link.start = 0;
  //dev->conf.tx_start = 0;
  //dev_reset(dev);

  // Configure DMA burst mode
  dev->lc_conf.val = I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN;
  // Set address of DMA descriptor
  dev->out_link.addr = (uint32_t) dma_descriptor;
  // Start DMA operation
  dev->out_link.stop = 0;
  dev->out_link.start = 1;
  dev->conf.tx_start = 1;

  return ESP_OK;
}

i2s_dev_t* i2s_parallel_get_dev(i2s_port_t port) {
  if(port < I2S_NUM_0 || port >= I2S_NUM_MAX) {
    return NULL;
  }
  return I2S[port];
}

// Double buffering flipping
// Flip to a buffer: 0 for bufa, 1 for bufb
void i2s_parallel_flip_to_buffer(i2s_port_t port, int buffer_id) {

    if (i2s_state == NULL) {
      return; // :-()
    }

    lldesc_t *active_dma_chain;
    if (buffer_id == 0) {
        active_dma_chain=(lldesc_t*)&i2s_state->dmadesc_a[0];
    } else {
        active_dma_chain=(lldesc_t*)&i2s_state->dmadesc_b[0];
    }

    // setup linked list to refresh from new buffer (continuously) when the end of the current list has been reached
    i2s_state->dmadesc_a[i2s_state->desccount_a-1].qe.stqe_next = active_dma_chain;
    i2s_state->dmadesc_b[i2s_state->desccount_b-1].qe.stqe_next = active_dma_chain;

    // we're still shifting out the buffer, so it shouldn't be written to yet.
    previousBufferFree = false;
}

bool i2s_parallel_is_previous_buffer_free() {
    return previousBufferFree;
}

