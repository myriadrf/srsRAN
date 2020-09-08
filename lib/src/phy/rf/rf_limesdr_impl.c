/*
 * Copyright 2020 Software Radio Systems Limited/Lime MicroSystems
 *
 * This file is part of srsLTE.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
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
#include "rf_limesdr_impl.h"
#include "srslte/srslte.h"

#define HAVE_ASYNC_THREAD 1
#define FORCE_USB_FAN 0
#define DISABLE_CH_AFTER_CLOSE 1
#define ALLOW_SUPPRESS_STDOUT 1

#define CALIBRATE_GFIR 4
#define CALIBRATE_FILTER 2
#define CALIBRATE_IQDC 1

typedef struct {
  char*         devname;
  lms_device_t* device;

  lms_stream_t rxStream[SRSLTE_MAX_PORTS];
  lms_stream_t txStream[SRSLTE_MAX_PORTS];
  size_t       num_rx_channels;
  size_t       num_tx_channels;
  bool         tx_stream_active;
  bool         rx_stream_active;
  bool         rx_first_shot;
  bool         use_12bit_format;
  int16_t*     rx_buffer[SRSLTE_MAX_PORTS]; // used only when format=I12
  int16_t*     tx_buffer[SRSLTE_MAX_PORTS]; // used only when format=I12

  bool             config_file;
  int              calibrate;
  double           tx_rate;
  double           rx_rate;
  size_t           dec_inter;
  srslte_rf_info_t info;

  srslte_rf_error_handler_t lime_error_handler;
  void*                     lime_error_handler_arg;

  bool      async_thread_running;
  pthread_t async_thread;
} rf_lime_handler_t;

cf_t zero_mem[64 * 1024];

#if HAVE_ASYNC_THREAD
static void log_overflow(rf_lime_handler_t* h)
{
  if (h->lime_error_handler) {
    srslte_rf_error_t error;
    bzero(&error, sizeof(srslte_rf_error_t));
    error.type = SRSLTE_RF_ERROR_OVERFLOW;
    h->lime_error_handler(h->lime_error_handler_arg, error);
  }
}
#endif

#if HAVE_ASYNC_THREAD
static void log_underflow(rf_lime_handler_t* h)
{
  if (h->lime_error_handler) {
    srslte_rf_error_t error;
    bzero(&error, sizeof(srslte_rf_error_t));
    error.type = SRSLTE_RF_ERROR_UNDERFLOW;
    h->lime_error_handler(h->lime_error_handler_arg, error);
  }
}
#endif

#if HAVE_ASYNC_THREAD
static void log_late(rf_lime_handler_t* h)
{
  if (h->lime_error_handler) {
    srslte_rf_error_t error;
    bzero(&error, sizeof(srslte_rf_error_t));
    error.type = SRSLTE_RF_ERROR_LATE;
    h->lime_error_handler(h->lime_error_handler_arg, error);
  }
}
#endif
// static void log_other(rf_lime_handler_t* h)
//{
//    if(h->lime_error_handler){
//        srslte_rf_error_t error;
//        bzero(&error, sizeof(srslte_rf_error_t));
//        error.type = SRSLTE_RF_ERROR_OTHER;
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

void rf_lime_register_error_handler(void* h, srslte_rf_error_handler_t new_handler, void* arg)
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
      printf("Error while receiving async information\n");
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
  if (rate < 5.76e6)
    return rate / 1.536;
  else if (rate == 15.36e6)
    return 15e6;
  else
    return rate / 1.152;
#endif
}

double get_analog_filter_bw(double rate)
{
  return get_channel_bw(rate) * 1.2;
}

const char* rf_lime_devname(void* h)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  return handler->devname;
}

int rf_lime_start_rx_stream(void* h, bool now)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (handler->rx_stream_active == false) {
    for (size_t i = 0; i < handler->num_rx_channels; i++)
      if (LMS_StartStream(&(handler->rxStream[i])) != 0) {
        printf("Error starting RX stream\n");
        return SRSLTE_ERROR;
      }
    handler->rx_stream_active = true;
  }
  return SRSLTE_SUCCESS;
}

int rf_lime_start_tx_stream(void* h)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (handler->tx_stream_active == false) {
    for (size_t i = 0; i < handler->num_tx_channels; i++)
      if (LMS_StartStream(&(handler->txStream[i])) != 0) {
        printf("Error starting TX stream\n");
        return SRSLTE_ERROR;
      }
    handler->tx_stream_active = true;
  }
  return SRSLTE_SUCCESS;
}

int rf_lime_stop_rx_stream(void* h)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (handler->rx_stream_active == true) {
    for (size_t i = 0; i < handler->num_rx_channels; i++)
      if (LMS_StopStream(&handler->rxStream[i]) != 0) {
        printf("Error stopping RX stream\n");
        return SRSLTE_ERROR;
      }
    handler->rx_stream_active = false;
  }
  return SRSLTE_SUCCESS;
}

int rf_lime_stop_tx_stream(void* h)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (handler->tx_stream_active == true) {
    for (size_t i = 0; i < handler->num_tx_channels; i++)
      if (LMS_StopStream(&handler->txStream[i]) != 0) {
        printf("Error stopping TX stream\n");
        return SRSLTE_ERROR;
      }
    handler->tx_stream_active = false;
  }
  return SRSLTE_SUCCESS;
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
    return SRSLTE_ERROR;
  }

  for (int i = 0; i < num_devices; i++) {
    printf("Found device #%d: ", (int)i);
    printf("%s\n", list[i]);
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
    printf("Error opening LimeSDR device\n");
    return SRSLTE_ERROR;
  }

  // Create handler
  rf_lime_handler_t* handler = (rf_lime_handler_t*)malloc(sizeof(rf_lime_handler_t));
  bzero(handler, sizeof(rf_lime_handler_t));
  *h                        = handler;
  handler->device           = sdr;
  handler->tx_stream_active = false;
  handler->rx_stream_active = false;
  handler->config_file      = false;
  handler->rx_first_shot    = true;

  // Set up device name
  if (strstr(list[lms_index], DEVNAME_MINI))
    handler->devname = DEVNAME_MINI;
  else if (strstr(list[lms_index], DEVNAME_USB))
    handler->devname = DEVNAME_USB;
  else
    handler->devname = "limesdr"; // Generic name

  // Check whether config file is available
  char  config_arg[]   = "config=";
  char  config_str[64] = {0};
  char* config_ptr     = strstr(args, config_arg);
  if (config_ptr) {
    copy_subdev_string(config_str, config_ptr + strlen(config_arg));
    printf("Loading config file %s\n", config_str);
    handler->config_file = true;
    if (LMS_LoadConfig(handler->device, config_str) != 0) {
      printf("Failed to load config file, continuing with normal configuration\n");
      handler->config_file = false;
    }
    remove_substring(args, config_arg);
    remove_substring(args, config_str);
  }

  // Initialize the device
  if (!handler->config_file) {
    printf("Initializing limesdr device\n");
    if (LMS_Init(sdr) != 0) {
      printf("Failed to ini LimeSDR device\n");
      return SRSLTE_ERROR;
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
      printf("failed to set external clock\n");
      return SRSLTE_ERROR;
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
  handler->num_rx_channels   = SRSLTE_MIN(num_available_channels, num_requested_channels);
  handler->num_tx_channels   = SRSLTE_MIN(num_available_channels, num_requested_channels);

  // Enable required channels
  if (num_available_channels > 0 && num_requested_channels > 0 && !handler->config_file) {
    for (size_t ch = 0; ch < handler->num_rx_channels; ch++) {
      if (LMS_EnableChannel(handler->device, LMS_CH_RX, ch, true) != 0) {
        printf("Failed to enable LimeSDR RX channel %d\n", (int)ch);
        return SRSLTE_ERROR;
      }
    }

    for (size_t ch = 0; ch < handler->num_tx_channels; ch++) {
      if (LMS_EnableChannel(handler->device, LMS_CH_TX, ch, true) != 0) {
        printf("Failed to enable LimeSDR TX channel %d\n", (int)ch);
        return SRSLTE_ERROR;
      }
    }
  }

  // Stream format setup
  handler->use_12bit_format = false;
  if (strstr(args, "format=i12")) {
    printf("Using 12 bit format");
    handler->use_12bit_format = true;
    remove_substring(args, "format=i12");
  } else if (strstr(args, "format=f32")) {
    printf("Using f32 format\n");
    remove_substring(args, "format=f32");
  } else if (strstr(args, "format=")) {
    printf("Wrong stream format, continuing with f32\n");
    remove_substring(args, "format=");
  }

  // Set up streamers
  for (uint16_t ch = 0; ch < handler->num_rx_channels; ch++) {
    printf("Setup RX stream %d\n", (int)ch);
    handler->rxStream[ch].channel             = ch;
    handler->rxStream[ch].fifoSize            = 256 * 1024;
    handler->rxStream[ch].throughputVsLatency = 0.3;
    handler->rxStream[ch].dataFmt             = handler->use_12bit_format ? LMS_FMT_I12 : LMS_FMT_F32;
    handler->rxStream[ch].isTx = false;
    if (LMS_SetupStream(handler->device, &(handler->rxStream[ch])) != 0) {
      printf("Failed to set up RX stream\n");
      return SRSLTE_ERROR;
    }
  }

  for (uint16_t ch = 0; ch < handler->num_rx_channels; ch++) {
    printf("Setup TX stream %d\n", (int)ch);
    handler->txStream[ch].channel             = ch;
    handler->txStream[ch].fifoSize            = 256 * 1024;
    handler->txStream[ch].throughputVsLatency = 0.3;
    handler->txStream[ch].dataFmt             = handler->use_12bit_format ? LMS_FMT_I12 : LMS_FMT_F32;
    handler->txStream[ch].isTx                = true;
    if (LMS_SetupStream(handler->device, &(handler->txStream[ch])) != 0) {
      printf("Failed to set up TX stream\n");
      return SRSLTE_ERROR;
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
          break;
        }
      }
    }

    // TX antenna
    if (txant_ptr) {
      copy_subdev_string(txant_str, txant_ptr + strlen(rxant_arg));
      // Find the required path
      for (int i = 0; i < num_tx_antennas; i++) {
        if (strstr(txant_str, tx_ant_list[i])) {
          ant_tx_path = i;
          break;
        }
      }
    }

    for (size_t i = 0; i < handler->num_tx_channels; i++)
      if (LMS_SetAntenna(sdr, LMS_CH_TX, i, ant_tx_path) != 0) {
        printf("Failed to set tx antenna\n");
        return SRSLTE_ERROR;
      }
    for (size_t i = 0; i < handler->num_rx_channels; i++)
      if (LMS_SetAntenna(sdr, LMS_CH_RX, i, ant_rx_path) != 0) {
        printf("Failed to set tx antenna\n");
        return SRSLTE_ERROR;
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
    if (LMS_WriteCustomBoardParam(handler->device, 0, tcxo_val, "") != 0) {
      printf("Failed to set TCXO value\n");
    }
    remove_substring(args, tcxo_str);
    remove_substring(args, tcxo_str);
  }

  // Set up async_thread
#if HAVE_ASYNC_THREAD
  handler->async_thread_running = true;
  if (pthread_create(&handler->async_thread, NULL, async_thread, handler)) {
    printf("pthread_create error\n");
    return SRSLTE_ERROR;
  }
#endif

  return SRSLTE_SUCCESS;
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

  for (size_t i = 0; i < handler->num_rx_channels; i++)
    if (handler->rx_buffer[i])
      free(handler->rx_buffer[i]);
  for (size_t i = 0; i < handler->num_tx_channels; i++)
    if (handler->tx_buffer[i])
      free(handler->tx_buffer[i]);

  free(handler);
  return SRSLTE_SUCCESS;
}

double rf_lime_set_rx_srate(void* h, double rate)
{
  rf_lime_handler_t* handler       = (rf_lime_handler_t*)h;
  bool               stream_active = handler->rx_stream_active;

  if (stream_active) {
    rf_lime_stop_rx_stream(handler);
  }

  if (LMS_SetSampleRateDir(handler->device, LMS_CH_RX, rate, handler->dec_inter) != 0) {
    printf("Failed to set RX sampling rate\n");
    return SRSLTE_ERROR;
  }

  double srate;
  if (LMS_GetSampleRate(handler->device, false, 0, &srate, NULL) != 0) {
    printf("Failed to get RX sampling rate\n");
    return SRSLTE_ERROR;
  }

  handler->rx_rate = srate;
  printf("RX sampling rate: %.2f\n", rate / 1e6);

  if (stream_active) {
    rf_lime_start_rx_stream(handler, true);
  }

  // Set up filters and calibration
  if (handler->calibrate & CALIBRATE_FILTER) {
    double analog_bw = get_analog_filter_bw(rate);
    printf("Setting analog RX LPF BW to: %.2f\n", analog_bw / 1e6);
    for (size_t i = 0; i < handler->num_rx_channels; i++) {
      if (LMS_SetLPFBW(handler->device, LMS_CH_RX, i, analog_bw) != 0) {
        printf("Failed to set analog RX LPF\n");
      }
    }
  }

  if (handler->calibrate & CALIBRATE_GFIR) {
    double digital_bw = get_channel_bw(rate);
    printf("Setting digital RX LPF BW to: %.2f\n", digital_bw / 1e6);
    for (size_t i = 0; i < handler->num_rx_channels; i++) {
      if (LMS_SetGFIRLPF(handler->device, LMS_CH_RX, i, true, digital_bw) != 0) {
        printf("Failed to set digital RX LPF\n");
      }
    }
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

  if (LMS_SetSampleRateDir(handler->device, LMS_CH_TX, rate, handler->dec_inter) != 0) {
    printf("Failed to set TX sampling rate\n");
    return SRSLTE_ERROR;
  }

  double srate;
  if (LMS_GetSampleRate(handler->device, true, 0, &srate, NULL) != 0) {
    printf("Failed to get TX sampling rate\n");
    return SRSLTE_ERROR;
  }

  printf("TX sampling rate: %.2f\n", srate / 1e6);
  handler->tx_rate = srate;

  if (handler->calibrate & CALIBRATE_FILTER) {
    for (size_t i = 0; i < handler->num_tx_channels; i++) {
      // Keep TX LPF open no matter the sampling rate
      if (LMS_SetLPFBW(handler->device, LMS_CH_TX, i, 30e6) != 0) {
        printf("Failed to disable analog TX LPF\n");
      }
    }
  }

  if (handler->calibrate & CALIBRATE_GFIR) {
    double digital_bw = get_channel_bw(rate);
    printf("Setting digital TX LPF BW to: %.2f\n", digital_bw / 1e6);
    for (size_t i = 0; i < handler->num_tx_channels; i++) {
      if (LMS_SetGFIRLPF(handler->device, LMS_CH_TX, i, true, digital_bw) != 0) {
        printf("Failed to set digital TX(%lu) LPF\n", i);
      }
    }
  }
  if (stream_active) {
    rf_lime_start_tx_stream(handler);
  }
  return srate;
}

double rf_lime_set_rx_gain(void* h, double gain)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (!handler->config_file) {
    for (size_t i = 0; i < handler->num_rx_channels; i++)
      if (LMS_SetGaindB(handler->device, false, 0, (unsigned)gain) != 0) {
        printf("Failed to set rx gain\n");
        return SRSLTE_ERROR;
      }
  } else {
    printf("Setting RX gain skipped\n");
  }

  unsigned actual_gain = 0;
  if (LMS_GetGaindB(handler->device, false, 0, &actual_gain) != 0) {
    printf("Failed get rx gain\n");
    return SRSLTE_ERROR;
  }
  printf("Actual RX gain: %u dB\n", actual_gain);
  return (double)actual_gain;
}

double rf_lime_set_tx_gain(void* h, double gain)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (!handler->config_file) {
    for (size_t i = 0; i < handler->num_tx_channels; i++)
      if (LMS_SetGaindB(handler->device, true, i, (unsigned)gain) != 0) {
        printf("Failed to set tx gain\n");
        return SRSLTE_ERROR;
      }
  } else {
    printf("Setting TX gain skipped\n");
  }

  unsigned actual_gain = 0;
  if (LMS_GetGaindB(handler->device, true, 0, &actual_gain) != 0) {
    printf("Failed get tx gain\n");
    return SRSLTE_ERROR;
  }
  printf("Actual TX gain: %u dB\n", actual_gain);
  return (double)actual_gain;
}

double rf_lime_get_rx_gain(void* h)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  unsigned           gain    = 0;
  if (LMS_GetGaindB(handler->device, false, 0, &gain) != 0) {
    printf("Failed get gain\n");
    return SRSLTE_ERROR;
  }
  return (double)gain;
}

double rf_lime_get_tx_gain(void* h)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  unsigned           gain    = 0;
  if (LMS_GetGaindB(handler->device, true, 0, &gain) != 0) {
    printf("Failed get gain\n");
    return SRSLTE_ERROR;
  }
  return (double)gain;
}

srslte_rf_info_t* rf_lime_get_info(void* h)
{
  srslte_rf_info_t* info = NULL;
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
    printf("Failed to set RX LO frequency\n");
    return SRSLTE_ERROR;
  }

  if (handler->calibrate & CALIBRATE_IQDC) {
    double bandwidth = get_channel_bw(handler->rx_rate);
    printf("Calibrating RX channel: %u, BW: %.2f\n", ch, bandwidth / 1e6);
    if (LMS_Calibrate(handler->device, LMS_CH_RX, ch, bandwidth, 0) != 0) {
      printf("Failed to calibrate RX channel :%u\n", ch);
    }
  }

  double actual_freq = 0.0;
  if (LMS_GetLOFrequency(handler->device, LMS_CH_RX, ch, &actual_freq) != 0) {
    printf("Failed to get LO frequency\n");
    return SRSLTE_ERROR;
  }

  return actual_freq;
}

double rf_lime_set_tx_freq(void* h, uint32_t ch, double freq)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  if (LMS_SetLOFrequency(handler->device, LMS_CH_TX, ch, freq) != 0) {
    printf("Failed to set RX LO frequency\n");
    return SRSLTE_ERROR;
  }

  if (handler->calibrate & CALIBRATE_IQDC) {
    double bandwidth = get_channel_bw(handler->tx_rate);
    printf("Calibrating TX channel: %u, BW: %.2f\n", ch, bandwidth / 1e6);
    if (LMS_Calibrate(handler->device, LMS_CH_TX, ch, bandwidth, 0) != 0) {
      printf("Failed to calibrate TX channel :%u\n", ch);
    }
  }

  double actual_freq = 0.0;
  if (LMS_GetLOFrequency(handler->device, LMS_CH_TX, ch, &actual_freq) != 0) {
    printf("Failed to get LO frequency\n");
    return SRSLTE_ERROR;
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
                                 void*    data[SRSLTE_MAX_PORTS],
                                 uint32_t nsamples,
                                 bool     blocking,
                                 time_t*  secs,
                                 double*  frac_secs)
{
  rf_lime_handler_t* handler = (rf_lime_handler_t*)h;
  lms_stream_meta_t  meta;
  uint32_t           num_total_samples           = 0;
  int                trials                      = 0;
  int                ret[SRSLTE_MAX_PORTS]       = {0};
  void*              buffs_ptr[SRSLTE_MAX_PORTS] = {0};

  if (handler->rx_first_shot) {
    handler->rx_first_shot = false;
    if (handler->use_12bit_format)
      for (size_t ch = 0; ch < handler->num_rx_channels; ch++)
        handler->rx_buffer[ch] =
            (int16_t*)malloc(32768 * 2 * sizeof(int16_t)); // allocate memory for 32768 samples (arbitraly chosen)
  }

  do {
    for (size_t ch = 0; ch < handler->num_rx_channels; ch++) {
      if (handler->use_12bit_format) {
        buffs_ptr[ch] = &handler->rx_buffer[ch][num_total_samples];
      } else {
        cf_t* data_c  = (cf_t*)data[ch];
        buffs_ptr[ch] = &data_c[num_total_samples];
      }
    }

    uint32_t num_samples_left = nsamples - num_total_samples;

    for (size_t i = 0; i < handler->num_rx_channels; i++) {
      ret[i] = LMS_RecvStream(&handler->rxStream[i], buffs_ptr[i], num_samples_left, &meta, 100);
      if (i > 0 && ret[0] != ret[i]) {
        printf("LMS_RecvStream misaligned channel data\n");
        return SRSLTE_ERROR;
      }
    }

    if (ret[0] < 0) {
      printf("LMS_RecvStream error\n");
      exit(-1);
      return SRSLTE_ERROR;
    } else {
      if (secs != NULL && frac_secs != NULL && num_total_samples == 0) {
        timestamp_to_secs(handler->rx_rate, meta.timestamp, secs, frac_secs);
      }

      // convert I12 to floats
      if (handler->use_12bit_format) {
        for (size_t ch = 0; ch < handler->num_rx_channels; ch++) {
          for (size_t i = 0; i < ret[0] * 2; i++) {
            ((float*)data[ch])[i] = ((int16_t*)buffs_ptr[ch])[i] / 2048.0f;
          }
        }
      }

      num_total_samples += ret[0];
    }

    trials++;
  } while (num_total_samples < nsamples && trials < 100);

  // TODO: handle this better
  if (trials == 100)
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
  void* _data[SRSLTE_MAX_PORTS] = {data, zero_mem, zero_mem, zero_mem};
  return rf_lime_send_timed_multi(
      h, _data, nsamples, secs, frac_secs, has_time_spec, blocking, is_start_of_burst, is_end_of_burst);
}

int rf_lime_send_timed_multi(void*  h,
                             void*  data[SRSLTE_MAX_PORTS],
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
  int                ret[SRSLTE_MAX_CHANNELS] = {0};

  meta.waitForTimestamp   = false;
  meta.flushPartialPacket = false;
  meta.timestamp          = 0;

  if (!handler->tx_stream_active) {
    if (handler->use_12bit_format)
      for (size_t ch = 0; ch < handler->num_tx_channels; ch++)
        handler->tx_buffer[ch] = (int16_t*)malloc(32768 * 2 * sizeof(int16_t)); // allocate memory for 32768 samples
    rf_lime_start_tx_stream(handler);
  }

  if (has_time_spec) {
    meta.waitForTimestamp   = true;
    meta.flushPartialPacket = is_end_of_burst;

    srslte_timestamp_t time = {secs, frac_secs};
    meta.timestamp          = srslte_timestamp_uint64(&time, handler->tx_rate);
  }

  void* buffs_ptr[SRSLTE_MAX_CHANNELS] = {};
  for (size_t ch = 0; ch < handler->num_tx_channels; ch++) {
    // Convert data to I12 if needed
    if (handler->use_12bit_format) {
      for (int i = 0; i < nsamples * 2; i++) {
        handler->tx_buffer[ch][i] = ((float*)data[ch])[i] * 2048.0f;
      }
      buffs_ptr[ch] = handler->tx_buffer[ch];
    } else {
      buffs_ptr[ch] = data[ch];
    }
  }

  do {
    uint32_t num_samples_left = nsamples - num_total_samples;

    for (size_t ch = 0; ch < handler->num_tx_channels; ch++) {
      buffs_ptr[ch] = (cf_t*)buffs_ptr[ch] + ret[0];
    }

    for (size_t ch = 0; ch < handler->num_tx_channels; ch++) {
      ret[ch] = LMS_SendStream(&handler->txStream[ch], buffs_ptr[ch], num_samples_left, &meta, 100);
      if (ch > 0 && ret[0] != ret[ch]) {
        printf("LMS_SendStream misaligned channel data\n");
        return SRSLTE_ERROR;
      }
    }

    if (ret[0] < 0) {
      printf("LMS_SendStream error\n");
      exit(-1);
      return SRSLTE_ERROR;
    } else {
      if (has_time_spec)
        meta.timestamp += ret[0];
      num_total_samples += ret[0];
    }

    trials++;
  } while (num_total_samples < nsamples && trials < 100);

  if (trials == 100) {
    printf("Too many trials\n");
  }
  return num_total_samples;
}
