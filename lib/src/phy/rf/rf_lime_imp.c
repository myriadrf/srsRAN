/**
 * Copyright 2013-2022 Software Radio Systems Limited 
 * Copyright 2020-2002 Lime MicroSystems Ltd
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <LimeSuite.h>
#include <pthread.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "rf_helper.h"
#include "rf_plugin.h"
#include "rf_lime_imp.h"
#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/common/timestamp.h"
#include "srsran/phy/utils/debug.h"
#include "srsran/phy/utils/vector.h"

#define HAVE_ASYNC_THREAD 1
#define FORCE_USB_FAN 0
#define DISABLE_CH_AFTER_CLOSE 1
#define ALLOW_SUPPRESS_STDOUT 1

#define CALIBRATE_GFIR 4
#define CALIBRATE_FILTER 2
#define CALIBRATE_IQDC 1

#define DIGITAL_GFIR_COEFF 0.95
#define ANALOG_LPF_COEFF 1

typedef struct {
  const char*   devname;
  lms_device_t* device;

  lms_stream_t rxStream[SRSRAN_MAX_PORTS];
  lms_stream_t txStream[SRSRAN_MAX_PORTS];
  size_t       num_rx_channels;
  size_t       num_tx_channels;
  bool         tx_stream_active;
  bool         rx_stream_active;

  bool             config_file;
  int              calibrate;
  bool             need_tx_cal;
  bool             need_rx_cal;
  double           tx_rate;
  double           rx_rate;
  size_t           dec_inter;
  srsran_rf_info_t info;

  srsran_rf_error_handler_t lime_error_handler;
  void*                     lime_error_handler_arg;

  bool      async_thread_running;
  pthread_t async_thread;
} rf_lime_handler_t;

static cf_t zero_mem[64 * 1024];

#if HAVE_ASYNC_THREAD
static void log_overflow(rf_lime_handler_t* h)
{
  if (h->lime_error_handler) {
    srsran_rf_error_t error;
    bzero(&error, sizeof(srsran_rf_error_t));
    error.type = SRSRAN_RF_ERROR_OVERFLOW;
    h->lime_error_handler(h->lime_error_handler_arg, error);
  }
}
#endif

#if HAVE_ASYNC_THREAD
static void log_underflow(rf_lime_handler_t* h)
{
  if (h->lime_error_handler) {
    srsran_rf_error_t error;
    bzero(&error, sizeof(srsran_rf_error_t));
    error.type = SRSRAN_RF_ERROR_UNDERFLOW;
    h->lime_error_handler(h->lime_error_handler_arg, error);
  }
}
#endif

#if HAVE_ASYNC_THREAD
static void log_late(rf_lime_handler_t* h)
{
  if (h->lime_error_handler) {
    srsran_rf_error_t error;
    bzero(&error, sizeof(srsran_rf_error_t));
    error.type = SRSRAN_RF_ERROR_LATE;
    h->lime_error_handler(h->lime_error_handler_arg, error);
  }
}
#endif
// static void log_other(rf_lime_handler_t* h)
//{
//    if(h->lime_error_handler){
//        srsran_rf_error_t error;
//        bzero(&error, sizeof(srsran_rf_error_t));
//        error.type = SRSRAN_RF_ERROR_OTHER;
//        h->lime_error_handler(h->lime_error_handler_arg, error);
//    }
//}

void rf_lime_suppress_handler(int lvl, const char* msg)
{
  // do nothing
}

void rf_lime_suppress_stdout(void* h)
{
#if ALLOW_SUPPRESS_STDOUT
  LMS_RegisterLogHandler(rf_lime_suppress_handler);
#endif
}

void rf_lime_register_error_handler(void* h, srsran_rf_error_handler_t new_handler, void* arg)
{
  rf_lime_handler_t* handler      = (rf_lime_handler_t*)h;
  handler->lime_error_handler     = new_handler;
  handler->lime_error_handler_arg = arg;
}

#if HAVE_ASYNC_THREAD
static void* async_thread(void* h)
{
  rf_lime_handler_t*  handler = (rf_lime_handler_t*)h;
  lms_stream_status_t tx_status;
  while (handler->async_thread_running) {
    if (LMS_GetStreamStatus(&handler->txStream[0], &tx_status) != 0) {
      printf("LMS_GetStreamStatus: Error while receiving async information\n");
      handler->async_thread_running = false;
      return NULL;
    }

    for (int i = 0; i < tx_status.overrun; i++)
      log_overflow(handler);

    for (int i = 0; i < tx_status.underrun; i++)
      log_underflow(handler);

    for (int i = 0; i < tx_status.droppedPackets; i++)
      log_late(handler);

    sleep(1);
  }
  return NULL;
}
#endif

// PRB  non standard standard
// 6    1.92e6       1.92e6
// 15   3.84e6       3.84e6
// 25   5.76e6       7.68e6
// 50   11.52e6      15.36e6
// 75   15.36e6      23.04e6
// 100  23.04e6      30.72e6
double get_channel_bw(double rate)
{
#ifdef FORCE_STANDARD_RATE
  return rate / 1.536;
#else
  
  double rounded_rate = round(rate / 100) * 100;
  if (rate < 5.76e6)
    return rate / 1.536;
  else if (rounded_rate == 15.36e6)
    return 15e6;
  else
    return rounded_rate / 1.152;
#endif
}

double get_analog_filter_bw(double rate)
{
  return get_channel_bw(rate) * ANALOG_LPF_COEFF; // 1.2;
}

double get_digital_filter_bw(double rate)
{
  return get_channel_bw(rate) * DIGITAL_GFIR_COEFF;
}

const char* rf_lime_devname(void* h)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  return handler->devname;
}

int rf_lime_start_tx_stream(void* h)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (handler->tx_stream_active == false) {
    if (handler->calibrate & CALIBRATE_IQDC && handler->need_tx_cal) {
      bool streamActive = handler->rx_stream_active;
      if(streamActive) {
        rf_lime_stop_rx_stream(h);
      }
      double bandwidth = get_channel_bw(handler->tx_rate);
      double cal_bw    = bandwidth > 2.5e6 ? bandwidth : 2.5e6;
      for(size_t ch = 0; ch < handler->num_tx_channels; ch++) {
        printf("Calibrating TX channel: %lu, BW: %.2f\n", ch, cal_bw / 1e6);
        if (LMS_Calibrate(handler->device, LMS_CH_TX, ch, cal_bw, 0) != 0) {
          printf("LMS_Calibrate: Failed to calibrate TX channel :%lu\n", ch);
        }
      }
      if(streamActive) {
        rf_lime_start_rx_stream(h,true);
      }
      handler->need_tx_cal = false;
    }

    for (size_t i = 0; i < handler->num_tx_channels; i++) {
      if (LMS_StartStream(&(handler->txStream[i])) != 0) {
        printf("LMS_StartStream: Error starting TX stream\n");
        return SRSRAN_ERROR;
      }
    }
    handler->tx_stream_active = true;
  }
  return SRSRAN_SUCCESS;
}

int rf_lime_stop_tx_stream(void* h)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (handler->tx_stream_active == true) {
    for (size_t i = 0; i < handler->num_tx_channels; i++)
      if (LMS_StopStream(&handler->txStream[i]) != 0) {
        printf("LMS_StopStream: Error stopping TX stream\n");
        return SRSRAN_ERROR;
      }
    handler->tx_stream_active = false;
  }
  return SRSRAN_SUCCESS;
}

int rf_lime_start_rx_stream(void* h, bool now)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (handler->rx_stream_active == false) {
    if (handler->calibrate & CALIBRATE_IQDC && handler->need_rx_cal) {
      bool streamActive = handler->tx_stream_active;
      if(streamActive) {
        rf_lime_stop_tx_stream(h);
      }
      double bandwidth = get_channel_bw(handler->rx_rate);
      double cal_bw    = bandwidth > 2.5e6 ? bandwidth : 2.5e6;
      for(size_t ch = 0; ch < handler->num_rx_channels; ch++) {
        printf("Calibrating RX channel: %lu, BW: %.2f\n", ch, cal_bw / 1e6);
        if (LMS_Calibrate(handler->device, LMS_CH_RX, ch, cal_bw, 0) != 0) {
          printf("LMS_Calibrate: Failed to calibrate RX channel :%lu\n", ch);
        }
      }
      if(streamActive) {
        rf_lime_start_tx_stream(h);
      }
      handler->need_rx_cal = false;
    }

    for (size_t i = 0; i < handler->num_rx_channels; i++) {
      if (LMS_StartStream(&(handler->rxStream[i])) != 0) {
        printf("LMS_StartStream: Error starting RX stream\n");
        return SRSRAN_ERROR;
      }
    }
    handler->rx_stream_active = true;
  }
  return SRSRAN_SUCCESS;
}

int rf_lime_stop_rx_stream(void* h)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (handler->rx_stream_active == true) {
    for (size_t i = 0; i < handler->num_rx_channels; i++)
      if (LMS_StopStream(&handler->rxStream[i]) != 0) {
        printf("LMS_StopStream: Error stopping RX stream\n");
        return SRSRAN_ERROR;
      }
    handler->rx_stream_active = false;
  }
  return SRSRAN_SUCCESS;
}

void rf_lime_flush_buffer(void* h)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  int                n;
  cf_t               tmp[1024];
  do {
    n = 0;
    for (size_t i = 0; i < handler->num_rx_channels; i++)
      n += LMS_RecvStream(&handler->rxStream[i], tmp, 1024, NULL, 100);
  } while (n > 0);
}

bool rf_lime_has_rssi(void* h)
{
  return false;
}

float rf_lime_get_rssi(void* h)
{
  return 0.0;
}

int rf_lime_open_multi(char* args, void** h, uint32_t num_requested_channels)
{
  lms_info_str_t list[16]    = {0};
  int            num_devices = LMS_GetDeviceList(list);
  printf("Number of requested channels: %d\n", num_requested_channels);
  if (num_devices == 0) {
    printf("No Lime devices found.\n");
    return SRSRAN_ERROR;
  }

  for (int i = 0; i < num_devices; i++) {
    printf("Found device #%d: ", (int)i);
    printf("%s\n", list[i]);
  }

  if (args == NULL) {
    args = "";
  }
  // Open device
  lms_device_t* sdr               = NULL;
  int           lms_index         = 0;
  char          dev_index_arg[]   = "index=";
  char          dev_index_str[64] = {0};
  char*         dev_index_ptr     = strstr(args, dev_index_arg);

  if (dev_index_ptr) {
    copy_subdev_string(dev_index_str, dev_index_ptr + strlen(dev_index_arg));
    lms_index = atoi(dev_index_str);
    if (lms_index >= num_devices) {
      printf("Requested index (%d) not available\n"
             "Using first device in the list\n",
             lms_index);
      lms_index = 0;
    }
    // TODO: fix string here
    // keep argument as it can remove other numbers
    remove_substring(args, dev_index_arg);
  }

  if (LMS_Open(&sdr, list[lms_index], NULL) != 0) {
    printf("LMS_Open: Error opening LimeSDR device\n");
    return SRSRAN_ERROR;
  }

  // Create handler
  rf_lime_handler_t* handler = (rf_lime_handler_t*)malloc(sizeof(rf_lime_handler_t));
  bzero(handler, sizeof(rf_lime_handler_t));
  *h                        = handler;
  handler->device           = sdr;
  handler->tx_stream_active = false;
  handler->rx_stream_active = false;
  handler->config_file      = false;  
  handler->need_rx_cal      = true;
  handler->need_tx_cal      = true;
  handler->devname          = LMS_GetDeviceInfo(sdr)->deviceName;

  // Check whether config file is available
  char  config_arg[]   = "config=";
  char  config_str[64] = {0};
  char* config_ptr     = strstr(args, config_arg);
  if (config_ptr) {
    copy_subdev_string(config_str, config_ptr + strlen(config_arg));
    printf("Loading config file %s\n", config_str);
    handler->config_file = true;
    if (LMS_LoadConfig(handler->device, config_str) != 0) {
      printf("LMS_LoadConfig: Failed to load config file, continuing with normal configuration\n");
      handler->config_file = false;
    }
    remove_substring(args, config_arg);
    remove_substring(args, config_str);
  }

  // Initialize the device
  if (!handler->config_file) {
    printf("Initializing limesdr device\n");
    if (LMS_Init(sdr) != 0) {
      printf("LMS_Init: Failed to initialize LimeSDR device\n");
      return SRSRAN_ERROR;
    }
  }

  // Reference clock setup
  char  refclk_arg[]   = "refclk=";
  char  refclk_str[64] = {0};
  char* refclk_ptr     = strstr(args, refclk_arg);
  if (refclk_ptr) {
    copy_subdev_string(refclk_str, refclk_ptr + strlen(refclk_arg));
    double freq = atof(refclk_str);
    printf("Setting reference clock to %.2f MHz\n", freq / 1e6);
    if (LMS_SetClockFreq(sdr, LMS_CLOCK_EXTREF, freq) != 0) {
      printf("LMS_SetClockFreq: failed to set external clock frequency\n");
      return SRSRAN_ERROR;
    }
    remove_substring(args, refclk_arg);
    remove_substring(args, refclk_str);
  }

#if FORCE_USB_FAN
  if (strcmp(handler->devname, DEVNAME_USB) == 0) {
    printf("Enabling fan\n");
    LMS_WriteFPGAReg(handler->device, 0xCC, 0x1);
    LMS_WriteFPGAReg(handler->device, 0xCD, 0x1);
  }
#endif

  int num_available_channels = LMS_GetNumChannels(sdr, false);
  handler->num_rx_channels   = SRSRAN_MIN(num_available_channels, num_requested_channels);
  handler->num_tx_channels   = SRSRAN_MIN(num_available_channels, num_requested_channels);

  // Enable required channels
  if (num_available_channels > 0 && num_requested_channels > 0 && !handler->config_file) {
    for (size_t ch = 0; ch < handler->num_rx_channels; ch++) {
      if (LMS_EnableChannel(handler->device, LMS_CH_RX, ch, true) != 0) {
        printf("LMS_EnableChannel: Failed to enable RX channel %d\n", (int)ch);
        return SRSRAN_ERROR;
      }
    }

    for (size_t ch = 0; ch < handler->num_tx_channels; ch++) {
      if (LMS_EnableChannel(handler->device, LMS_CH_TX, ch, true) != 0) {
        printf("LMS_EnableChannel: Failed to enable TX channel %d\n", (int)ch);
        return SRSRAN_ERROR;
      }
    }
  }

  // Stream format setup
  unsigned linkFormat = 0;
  if (strstr(args, "format=i12")) {
    printf("Using 12 bit link format\n");
    linkFormat = 2;
    remove_substring(args, "format=i12");
  } else if (strstr(args, "format=i16")) {
    printf("Using 16 bit link format\n");
    linkFormat = 1;
    remove_substring(args, "format=i16");
  } else if (strstr(args, "format=")) {
    printf("Wrong link format, continuing with default\n");
    remove_substring(args, "format=");
  }

  // Set up streamers
  for (uint16_t ch = 0; ch < handler->num_rx_channels; ch++) {
    printf("Setup RX stream %d\n", (int)ch);
    handler->rxStream[ch].channel             = ch;
    handler->rxStream[ch].fifoSize            = 256 * 1024;
    handler->rxStream[ch].throughputVsLatency = 0.3;
    handler->rxStream[ch].dataFmt             = LMS_FMT_F32;
    handler->rxStream[ch].linkFmt             = linkFormat;
    handler->rxStream[ch].isTx                = false;
    if (LMS_SetupStream(handler->device, &(handler->rxStream[ch])) != 0) {
      printf("LMS_SetupStream: Failed to set up RX stream\n");
      return SRSRAN_ERROR;
    }
  }

  for (uint16_t ch = 0; ch < handler->num_tx_channels; ch++) {
    printf("Setup TX stream %d\n", (int)ch);
    handler->txStream[ch].channel             = ch;
    handler->txStream[ch].fifoSize            = 256 * 1024;
    handler->txStream[ch].throughputVsLatency = 0.3;
    handler->txStream[ch].dataFmt             = LMS_FMT_F32;
    handler->txStream[ch].linkFmt             = linkFormat;
    handler->txStream[ch].isTx                = true;
    if (LMS_SetupStream(handler->device, &(handler->txStream[ch])) != 0) {
      printf("LMS_SetupStream: Failed to set up TX stream\n");
      return SRSRAN_ERROR;
    }
  }

  // Set up antennas
  lms_name_t rx_ant_list[16] = {0};
  int        num_rx_antennas = LMS_GetAntennaList(handler->device, LMS_CH_RX, 0, rx_ant_list);
  lms_name_t tx_ant_list[16] = {0};
  int        num_tx_antennas = LMS_GetAntennaList(handler->device, LMS_CH_TX, 0, tx_ant_list);
  // Skip antenna configuration if config file is loaded
  if (!handler->config_file) {
    // Default paths for > 1700 MHz
    size_t ant_rx_path = LMS_PATH_LNAH;
    size_t ant_tx_path = LMS_PATH_TX2;
    bool skip_rx_mini_path = false;
    bool skip_tx_mini_path = false;

    if (strcmp(handler->devname, DEVNAME_MINI) == 0) {
      skip_rx_mini_path = true;
      skip_tx_mini_path = true;
    }

    char  rxant_arg[]   = "rxant=";
    char  rxant_str[64] = {0};
    char* rxant_ptr     = strstr(args, rxant_arg);

    char  txant_arg[]   = "txant=";
    char  txant_str[64] = {0};
    char* txant_ptr     = strstr(args, txant_arg);
    
    // RX antenna
    if (rxant_ptr) {
      copy_subdev_string(rxant_str, rxant_ptr + strlen(rxant_arg));
      // Find the required path
      for (int i = 0; i < num_rx_antennas; i++) {
        if (strstr(rxant_str, rx_ant_list[i])) {
          ant_rx_path = i;
          skip_rx_mini_path = false;
          break;
        }
      }
    }

    // TX antenna
    if (txant_ptr) {
      copy_subdev_string(txant_str, txant_ptr + strlen(txant_arg));
      // Find the required path
      for (int i = 0; i < num_tx_antennas; i++) {
        if (strstr(txant_str, tx_ant_list[i])) {
          ant_tx_path = i;
          skip_tx_mini_path = false;
          break;
        }
      }
    }

    for (size_t i = 0; i < handler->num_tx_channels && !skip_tx_mini_path; i++) {
      if (LMS_SetAntenna(sdr, LMS_CH_TX, i, ant_tx_path) != 0) {
        printf("LMS_SetAntenna: Failed to set TX antenna\n");
        return SRSRAN_ERROR;
      }
    }
    for (size_t i = 0; i < handler->num_rx_channels && !skip_rx_mini_path; i++)
      if (LMS_SetAntenna(sdr, LMS_CH_RX, i, ant_rx_path) != 0) {
        printf("LMS_SetAntenna: Failed to set RX antenna\n");
        return SRSRAN_ERROR;
      }
  }

  // Print current used paths
  printf("RX antenna/s set to: %s\n", rx_ant_list[LMS_GetAntenna(handler->device, LMS_CH_RX, 0)]);
  printf("TX antenna/s set to: %s\n", tx_ant_list[LMS_GetAntenna(handler->device, LMS_CH_TX, 0)]);

  // Configure decimation/interpolation
  handler->dec_inter      = 0;
  char  dec_inter_arg[]   = "dec_inter=";
  char  dec_inter_str[64] = {0};
  char* dec_inter_ptr     = strstr(args, dec_inter_arg);
  if (dec_inter_ptr) {
    copy_subdev_string(dec_inter_str, dec_inter_ptr + strlen(dec_inter_arg));
    handler->dec_inter = atoi(dec_inter_str);
    printf("Setting decimation/interpolation to %lu\n", handler->dec_inter);
    remove_substring(args, dec_inter_arg);
    remove_substring(args, dec_inter_arg);
  }

  // Calibration
  handler->calibrate = CALIBRATE_FILTER | CALIBRATE_IQDC;
  char  cal_arg[]    = "cal=";
  char  cal_str[64]  = {0};
  char* cal_ptr      = strstr(args, cal_arg);
  if (cal_ptr) {
    copy_subdev_string(cal_str, cal_ptr + strlen(cal_arg));

    if (strstr(cal_str, "iq_dc"))
      handler->calibrate = CALIBRATE_IQDC;
    else if (strstr(cal_str, "filter"))
      handler->calibrate = CALIBRATE_FILTER;
    else if (strstr(cal_str, "all"))
      handler->calibrate = CALIBRATE_FILTER | CALIBRATE_IQDC;
    else if (strstr(cal_str, "none"))
      handler->calibrate = 0;

    remove_substring(args, cal_arg);
    remove_substring(args, cal_str);
  }

  // GFIR
  char  gfir_arg[]   = "gfir=";
  char  gfir_str[64] = {0};
  char* gfir_ptr     = strstr(args, gfir_arg);
  if (gfir_ptr) {
    copy_subdev_string(gfir_str, gfir_ptr + strlen(gfir_arg));

    if (strstr(gfir_str, "on") || strstr(gfir_str, "true"))
      handler->calibrate |= CALIBRATE_GFIR;

    remove_substring(args, gfir_arg);
    remove_substring(args, gfir_str);
  }

  // TCXO runtime parameter
  char  tcxo_arg[]   = "tcxo=";
  char  tcxo_str[64] = {0};
  char* tcxo_ptr     = strstr(args, tcxo_arg);
  if (tcxo_ptr) {
    copy_subdev_string(tcxo_str, tcxo_ptr + strlen(tcxo_arg));
    int tcxo_val = atoi(tcxo_str);
    printf("Setting TCXO value to %d\n", tcxo_val);
    if (LMS_WriteCustomBoardParam(handler->device, 0, tcxo_val, NULL) != 0) {
      printf("LMS_WriteCustomBoardParam: Failed to set TCXO value\n");
    }
    remove_substring(args, tcxo_str);
    remove_substring(args, tcxo_str);
  }

  if (strcmp(handler->devname, DEVNAME_CC) == 0) {
    LMS_WriteFPGAReg(handler->device, 0x18, 0x0); // TDD_Mode
    LMS_WriteFPGAReg(handler->device, 0x18, 0x0); // TDD_mode
    LMS_WriteFPGAReg(handler->device, 0x10, 0x1); // TDD_start_delay
    LMS_WriteFPGAReg(handler->device, 0x11, 0x1); // TDD_stop_delay
    LMS_WriteFPGAReg(handler->device, 0xCC, 0x0); // TDD_switch_mode
    LMS_WriteFPGAReg(handler->device, 0xCD, 0x0); // TDD_switch_dir

    uint16_t reg13 = 0;
    LMS_ReadFPGAReg(handler->device, 0x13, &reg13);
    reg13 &= ~((1 << 7) | (1 << 15) | (1 << 8) | (1 << 14)); // TX/RX enable mux
    LMS_WriteFPGAReg(handler->device, 0x13, reg13);
  }

  // Set up async_thread
#if HAVE_ASYNC_THREAD
  handler->async_thread_running = true;
  if (pthread_create(&handler->async_thread, NULL, async_thread, handler)) {
    printf("pthread_create error\n");
    return SRSRAN_ERROR;
  }
#endif

  return SRSRAN_SUCCESS;
}

int rf_lime_open(char* args, void** h)
{
  return rf_lime_open_multi(args, h, 1);
}

int rf_lime_close(void* h)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
#if HAVE_ASYNC_THREAD
  if (handler->async_thread_running) {
    handler->async_thread_running = false;
    pthread_join(handler->async_thread, NULL);
  }
#endif

  if (handler->rx_stream_active) {
    rf_lime_stop_rx_stream(handler);
  }
  if (handler->tx_stream_active) {
    rf_lime_stop_tx_stream(handler);
  }

  for (size_t i = 0; i < handler->num_rx_channels; i++)
    LMS_DestroyStream(handler->device, &handler->rxStream[i]);
  for (size_t i = 0; i < handler->num_tx_channels; i++)
    LMS_DestroyStream(handler->device, &handler->txStream[i]);

#if DISABLE_CH_AFTER_CLOSE
  for (size_t i = 0; i < handler->num_rx_channels; i++)
    LMS_EnableChannel(handler->device, LMS_CH_RX, i, false);

  for (size_t i = 0; i < handler->num_tx_channels; i++)
    LMS_EnableChannel(handler->device, LMS_CH_TX, i, false);
#endif // DISABLE_CH_AFTER_CLOSE

  LMS_Close(handler->device);
  free(handler);

  return SRSRAN_SUCCESS;
}

double rf_lime_set_rx_srate(void* h, double rate)
{
  rf_lime_handler_t* handler       = (rf_lime_handler_t*)h;
  bool               stream_active = handler->rx_stream_active;

  if (stream_active) {
    rf_lime_stop_rx_stream(handler);
  }

  if (LMS_SetSampleRateDir(handler->device, LMS_CH_RX, rate, handler->dec_inter) != 0) {
    printf("LMS_SetSampleRate: Failed to set RX sampling rate\n");
    return SRSRAN_ERROR;
  }

  double srate;
  if (LMS_GetSampleRate(handler->device, false, 0, &srate, NULL) != 0) {
    printf("LMS_GetSampleRate: Failed to get RX sampling rate\n");
    return SRSRAN_ERROR;
  }

  handler->rx_rate = srate;
  printf("RX sampling rate: %.2f\n", srate / 1e6);

  // Set up filters and calibration
  uint16_t reg13;
  if (strcmp(handler->devname, DEVNAME_CC)) {
    LMS_ReadFPGAReg(handler->device, 0x13, &reg13);
    LMS_WriteFPGAReg(handler->device, 0x13, reg13 | 0x60);
  }

  if (handler->calibrate & CALIBRATE_FILTER) {
    double filter_bw = get_analog_filter_bw(rate);
    double analog_bw = filter_bw > 1.5e6 ? filter_bw : 1.5e6;
    printf("Setting analog RX LPF BW to: %.2f\n", analog_bw / 1e6);
    for (size_t i = 0; i < handler->num_rx_channels; i++) {
      if (LMS_SetLPFBW(handler->device, LMS_CH_RX, i, analog_bw) != 0) {
        printf("LMS_SetLPFBW: Failed to set analog RX LPF\n");
      }
    }
  }

  if (handler->calibrate & CALIBRATE_GFIR) {
    double digital_bw = get_digital_filter_bw(rate);
    printf("Setting digital RX LPF BW to: %.2f\n", digital_bw / 1e6);
    for (size_t i = 0; i < handler->num_rx_channels; i++) {
      if (LMS_SetGFIRLPF(handler->device, LMS_CH_RX, i, true, digital_bw) != 0) {
        printf("LMS_SetGFIRLPF: Failed to set digital RX LPF\n");
      }
    }
  }

  if (strcmp(handler->devname, DEVNAME_CC)) {
    LMS_WriteFPGAReg(handler->device, 0x13, reg13);
  }

  if (stream_active) {
    rf_lime_start_rx_stream(handler, true);
  }

  return srate;
}

double rf_lime_set_tx_srate(void* h, double rate)
{
  rf_lime_handler_t* handler       = (rf_lime_handler_t*)h;
  bool               stream_active = handler->tx_stream_active;

  if (stream_active) {
    rf_lime_stop_tx_stream(handler);
  }
  
  if(LMS_SetSampleRate(handler->device, rate, handler->dec_inter) != 0) {
  //if (LMS_SetSampleRateDir(handler->device, LMS_CH_TX, rate, handler->dec_inter) != 0) {
    printf("LMS_SetSampleRate: Failed to set TX sampling rate\n");
    return SRSRAN_ERROR;
  }

  double srate;
  if (LMS_GetSampleRate(handler->device, true, 0, &srate, NULL) != 0) {
    printf("LMS_GetSampleRate: Failed to get TX sampling rate\n");
    return SRSRAN_ERROR;
  }

  printf("TX sampling rate: %.2f\n", srate / 1e6);
  handler->tx_rate = srate;

  // Set up filters and calibration
  uint16_t reg13;
  if (strcmp(handler->devname, DEVNAME_CC)) {
    LMS_ReadFPGAReg(handler->device, 0x13, &reg13);
    LMS_WriteFPGAReg(handler->device, 0x13, reg13 | 0x60);
  }

  if (handler->calibrate & CALIBRATE_FILTER) {
    double filter_bw = get_analog_filter_bw(rate);
    double analog_bw = filter_bw > 5e6 ? filter_bw : 5e6;
    printf("Setting analog TX LPF BW to: %.2f\n", analog_bw / 1e6);
    for (size_t i = 0; i < handler->num_tx_channels; i++) {
      if (LMS_SetLPFBW(handler->device, LMS_CH_TX, i, analog_bw) != 0) {
        printf("LMS_SetLPFBW: Failed to disable analog TX LPF\n");
      }
    }
  }

  if (handler->calibrate & CALIBRATE_GFIR) {
    double digital_bw = get_digital_filter_bw(rate);
    printf("Setting digital TX LPF BW to: %.2f\n", digital_bw / 1e6);
    for (size_t i = 0; i < handler->num_tx_channels; i++) {
      if (LMS_SetGFIRLPF(handler->device, LMS_CH_TX, i, true, digital_bw) != 0) {
        printf("LMS_SetGFIRLPF: Failed to set digital TX(%lu) LPF\n", i);
      }
    }
  }

  if (strcmp(handler->devname, DEVNAME_CC)) {
    LMS_WriteFPGAReg(handler->device, 0x13, reg13);
  }

  if (stream_active) {
    rf_lime_start_tx_stream(handler);
  }
  return srate;
}

int rf_lime_set_rx_gain(void* h, double gain)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (!handler->config_file) {
    for (size_t i = 0; i < handler->num_rx_channels; i++)
      if (LMS_SetGaindB(handler->device, false, 0, (unsigned)gain) != 0) {
        printf("LMS_SetGaindB: Failed to set RX gain\n");
        return SRSRAN_ERROR;
      }
  } else {
    printf("Setting RX gain skipped\n");
  }
  return SRSRAN_SUCCESS;
}

int rf_lime_set_rx_gain_ch(void* h, uint32_t ch, double gain)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (!handler->config_file) {
    if (LMS_SetGaindB(handler->device, false, ch, (unsigned)gain) != 0) {
      printf("LMS_SetGaindB: Failed to set RX gain\n");
      return SRSRAN_ERROR;
    }
  } else {
    printf("Setting RX gain skipped\n");
  }
  return SRSRAN_SUCCESS;
}

int rf_lime_set_tx_gain(void* h, double gain)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (!handler->config_file) {
    for (size_t i = 0; i < handler->num_tx_channels; i++)
      if (LMS_SetGaindB(handler->device, true, i, (unsigned)gain) != 0) {
        printf("LMS_SetGaindB: Failed to set TX gain\n");
        return SRSRAN_ERROR;
      }
  } else {
    printf("Setting TX gain skipped\n");
  }
  return SRSRAN_SUCCESS;
}

int rf_lime_set_tx_gain_ch(void* h, uint32_t ch, double gain)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (!handler->config_file) {
    if (LMS_SetGaindB(handler->device, true, ch, (unsigned)gain) != 0) {
      printf("LMS_SetGaindB: Failed to set RX gain\n");
      return SRSRAN_ERROR;
    }
  } else {
    printf("Setting RX gain skipped\n");
  }
  return SRSRAN_SUCCESS;
}

double rf_lime_get_rx_gain(void* h)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  unsigned           gain    = 0;
  if (LMS_GetGaindB(handler->device, false, 0, &gain) != 0) {
    printf("LMS_GetGaindB: Failed get gain\n");
    return SRSRAN_ERROR;
  }
  return (double)gain;
}

double rf_lime_get_tx_gain(void* h)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  unsigned           gain    = 0;
  if (LMS_GetGaindB(handler->device, true, 0, &gain) != 0) {
    printf("LMS_GetGaindB: Failed get gain\n");
    return SRSRAN_ERROR;
  }
  return (double)gain;
}

srsran_rf_info_t* rf_lime_get_info(void* h)
{
  srsran_rf_info_t* info = NULL;
  if (h) {
    rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
    info                       = &handler->info;
  }
  return info;
}

double rf_lime_set_rx_freq(void* h, uint32_t ch, double freq)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (LMS_SetLOFrequency(handler->device, LMS_CH_RX, ch, freq) != 0) {
    printf("LMS_SetLOFrequency: Failed to set RX LO frequency\n");
    return SRSRAN_ERROR;
  }

  double actual_freq = 0.0;
  if (LMS_GetLOFrequency(handler->device, LMS_CH_RX, ch, &actual_freq) != 0) {
    printf("LMS_GetLOFrequency: Failed to get LO frequency\n");
    return SRSRAN_ERROR;
  }

  return actual_freq;
}

double rf_lime_set_tx_freq(void* h, uint32_t ch, double freq)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (LMS_SetLOFrequency(handler->device, LMS_CH_TX, ch, freq) != 0) {
    printf("LMS_SetLOFrequency: Failed to set RX LO frequency\n");
    return SRSRAN_ERROR;
  }

  double actual_freq = 0.0;
  if (LMS_GetLOFrequency(handler->device, LMS_CH_TX, ch, &actual_freq) != 0) {
    printf("LMS_GetLOFrequency: Failed to get LO frequency\n");
    return SRSRAN_ERROR;
  }

  return actual_freq;
}

static void timestamp_to_secs(double rate, uint64_t timestamp, time_t* secs, double* frac_secs)
{
  double totalsecs = (double)timestamp / rate;
  if (secs && frac_secs) {
    *secs      = (time_t)totalsecs;
    *frac_secs = totalsecs - (*secs);
  }
}

void rf_lime_get_time(void* h, time_t* secs, double* frac_secs)
{
  rf_lime_handler_t*  handler = (rf_lime_handler_t*)h;
  lms_stream_status_t status;
  if (handler->rx_stream_active) {
    LMS_GetStreamStatus(&handler->rxStream[0], &status);
    timestamp_to_secs(handler->rx_rate, status.timestamp, secs, frac_secs);
  }
}

int rf_lime_recv_with_time_multi(void*    h,
                                 void**    data,
                                 uint32_t nsamples,
                                 bool     blocking,
                                 time_t*  secs,
                                 double*  frac_secs)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  lms_stream_meta_t  meta;
  uint32_t           num_total_samples           = 0;
  int                trials                      = 0;
  int                ret[SRSRAN_MAX_PORTS]       = {0};
  void*              buffs_ptr[SRSRAN_MAX_PORTS] = {0};

  do {
    for (size_t ch = 0; ch < handler->num_rx_channels; ch++) {
        cf_t* data_c  = (cf_t*)data[ch];
        buffs_ptr[ch] = &data_c[num_total_samples];
    }

    uint32_t num_samples_left = nsamples - num_total_samples;

    for (size_t i = 0; i < handler->num_rx_channels; i++) {
      ret[i] = LMS_RecvStream(&handler->rxStream[i], buffs_ptr[i], num_samples_left, &meta, 100);
      if (i > 0 && ret[0] != ret[i]) {
        printf("LMS_RecvStream: misaligned channel data\n");
        return SRSRAN_ERROR;
      }
    }

    if (ret[0] < 0) {
      printf("LMS_RecvStream error\n");
      exit(-1);
      return SRSRAN_ERROR;
    } else {
      if (secs != NULL && frac_secs != NULL && num_total_samples == 0) {
        timestamp_to_secs(handler->rx_rate, meta.timestamp, secs, frac_secs);
      }

      num_total_samples += ret[0];
    }

    trials++;
  } while (num_total_samples < nsamples && trials < 10);

  // TODO: handle this better
  if (trials == 10)
    printf("Too many RX trials\n");

  return num_total_samples;
}

int rf_lime_recv_with_time(void* h, void* data, uint32_t nsamples, bool blocking, time_t* secs, double* frac_secs)
{
  return rf_lime_recv_with_time_multi(h, &data, nsamples, blocking, secs, frac_secs);
}

int rf_lime_send_timed(void*  h,
                       void*  data,
                       int    nsamples,
                       time_t secs,
                       double frac_secs,
                       bool   has_time_spec,
                       bool   blocking,
                       bool   is_start_of_burst,
                       bool   is_end_of_burst)
{
  void* _data[SRSRAN_MAX_PORTS] = {data, zero_mem, zero_mem, zero_mem};
  return rf_lime_send_timed_multi(
      h, _data, nsamples, secs, frac_secs, has_time_spec, blocking, is_start_of_burst, is_end_of_burst);
}

int rf_lime_send_timed_multi(void*  h,
                             void*  data[SRSRAN_MAX_PORTS],
                             int    nsamples,
                             time_t secs,
                             double frac_secs,
                             bool   has_time_spec,
                             bool   blocking,
                             bool   is_start_of_burst,
                             bool   is_end_of_burst)
{

  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  lms_stream_meta_t  meta;
  int                num_total_samples        = 0;
  int                trials                   = 0;
  int                ret[SRSRAN_MAX_CHANNELS] = {0};

  meta.waitForTimestamp   = false;
  meta.flushPartialPacket = false;
  meta.timestamp          = 0;

  if (!handler->tx_stream_active) {
    rf_lime_start_tx_stream(handler);
  }

  if (has_time_spec) {
    meta.waitForTimestamp   = true;
    meta.flushPartialPacket = is_end_of_burst;

    srsran_timestamp_t time = {secs, frac_secs};
    meta.timestamp          = srsran_timestamp_uint64(&time, handler->tx_rate);
  }

  void* buffs_ptr[SRSRAN_MAX_CHANNELS] = {};
  for (size_t ch = 0; ch < handler->num_tx_channels; ch++) {
      buffs_ptr[ch] = data[ch];
  }

  do {
    uint32_t num_samples_left = nsamples - num_total_samples;

    for (size_t ch = 0; ch < handler->num_tx_channels; ch++) {
      buffs_ptr[ch] = (cf_t*)buffs_ptr[ch] + ret[0];
    }

    for (size_t ch = 0; ch < handler->num_tx_channels; ch++) {
      ret[ch] = LMS_SendStream(&handler->txStream[ch], buffs_ptr[ch], num_samples_left, &meta, 100);
      if (ch > 0 && ret[0] != ret[ch]) {
        printf("LMS_SendStream: misaligned channel data\n");
        return SRSRAN_ERROR;
      }
    }

    if (ret[0] < 0) {
      printf("LMS_SendStream error\n");
      exit(-1);
      return SRSRAN_ERROR;
    } else {
      if (has_time_spec)
        meta.timestamp += ret[0];
      num_total_samples += ret[0];
    }

    trials++;
  } while (num_total_samples < nsamples && trials < 10);

  if (trials == 10) {
    printf("Too many trials\n");
  }
  return num_total_samples;
}

rf_dev_t srsran_rf_dev_lime = {"lime",
                               rf_lime_devname,
                               rf_lime_start_rx_stream,
                               rf_lime_stop_rx_stream,
                               rf_lime_flush_buffer,
                               rf_lime_has_rssi,
                               rf_lime_get_rssi,
                               rf_lime_suppress_stdout,
                               rf_lime_register_error_handler,
                               rf_lime_open,
                               rf_lime_open_multi,
                               rf_lime_close,
                               rf_lime_set_rx_srate,
                               rf_lime_set_rx_gain,
                               rf_lime_set_rx_gain_ch,
                               rf_lime_set_tx_gain,
                               rf_lime_set_tx_gain_ch,
                               rf_lime_get_tx_gain,
                               rf_lime_get_rx_gain,
                               rf_lime_get_info,
                               rf_lime_set_rx_freq,
                               rf_lime_set_tx_srate,
                               rf_lime_set_tx_freq,
                               rf_lime_get_time,
                               NULL,
                               rf_lime_recv_with_time,
                               rf_lime_recv_with_time_multi,
                               rf_lime_send_timed,
                               .srsran_rf_send_timed_multi = rf_lime_send_timed_multi};

#ifdef ENABLE_RF_PLUGINS
int register_plugin(rf_dev_t** rf_api)
{
  if (rf_api == NULL) {
    return SRSRAN_ERROR;
  }
  *rf_api = &srsran_rf_dev_lime;
  return SRSRAN_SUCCESS;
}
#endif /* ENABLE_RF_PLUGINS */
