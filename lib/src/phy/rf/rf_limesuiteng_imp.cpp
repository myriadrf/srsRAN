/**
 * Copyright 2013-2022 Software Radio Systems Limited
 * Copyright 2020-2024 Lime MicroSystems Ltd
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

#include <string.h>
#include <string>
#include <sys/time.h>
#include <unistd.h>

#include "rf_helper.h"
#include "rf_limesuiteng_imp.h"
#include "rf_plugin.h"
#include "srsran/phy/common/phy_common.h"
#include "srsran/phy/common/timestamp.h"
#include "srsran/phy/utils/debug.h"
#include "srsran/phy/utils/vector.h"

#include "limesuiteng/LimePlugin.h"
#include "limesuiteng/limesuiteng.hpp"
using namespace lime;

typedef struct {
  const char*        devname;
  LimePluginContext* context;

  double                    tx_rate;
  double                    rx_rate;
  srsran_rf_error_handler_t lime_error_handler;
  void*                     lime_error_handler_arg;

} rf_limesuiteng_handler_t;

void rf_limesuiteng_register_error_handler(void* h, srsran_rf_error_handler_t new_handler, void* arg)
{
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  handler->lime_error_handler       = new_handler;
  handler->lime_error_handler_arg   = arg;
}

void rf_limesuiteng_suppress_stdout(void* h) {}

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

const char* rf_limesuiteng_devname(void* h)
{
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  return handler->devname;
}

int rf_limesuiteng_start_rx_stream(void* h, bool now)
{
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  LimePluginContext*        lime    = handler->context;
  int                       status  = LimePlugin_Start(lime);
  return status == 0 ? SRSRAN_SUCCESS : SRSRAN_ERROR_CANT_START;
}

int rf_limesuiteng_stop_rx_stream(void* h)
{
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  LimePluginContext*        lime    = handler->context;
  int                       status  = LimePlugin_Stop(lime);
  return status == 0 ? SRSRAN_SUCCESS : SRSRAN_ERROR;
}

void rf_limesuiteng_flush_buffer(void* h)
{
  // rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  // TODO: implement flush
}

bool rf_limesuiteng_has_rssi(void* h)
{
  return false;
}

float rf_limesuiteng_get_rssi(void* h)
{
  return 0.0;
}

static lime::LogLevel logVerbosity = lime::LogLevel::Debug;
static void           LogCallback(LogLevel lvl, const std::string& msg)
{
  if (lvl > logVerbosity)
    return;
  printf("%s\n", msg.c_str());
}

class srsRAN_ParamProvider : public LimeSettingsProvider
{
private:
  static std::string trim(const std::string& s)
  {
    std::string out = s;
    while (!out.empty() && std::isspace(out[0]))
      out = out.substr(1);
    while (!out.empty() && std::isspace(out[out.size() - 1]))
      out = out.substr(0, out.size() - 1);
    return out;
  }

  void argsToMap(const std::string& args)
  {
    std::map<std::string, std::string> kwmap;

    bool        inKey = true;
    std::string key, val;
    for (size_t i = 0; i < args.size(); i++) {
      const char ch = args[i];
      if (inKey) {
        if (ch == '=')
          inKey = false;
        else if (ch == ',')
          inKey = true;
        else
          key += ch;
      } else {
        if (ch == ',')
          inKey = true;
        else
          val += ch;
      }
      if ((inKey && !val.empty()) || ((i + 1) == args.size())) {
        key = trim(key);
        val = trim(val);
        if (!key.empty())
          kwmap[key] = val;
        key = "";
        val = "";
      }
    }
  }

public:
  srsRAN_ParamProvider(const char* args) : mArgs(args) { argsToMap(mArgs); }

  bool GetString(std::string& dest, const char* varname) override
  {
    auto iter = strings.find(varname);
    if (iter == strings.end())
      return false;

    dest = iter->second;
    return true;
  }

  bool GetDouble(double& dest, const char* varname) override
  {
    auto iter = numbers.find(varname);
    if (iter == numbers.end())
      return false;

    dest = iter->second;
    return true;
  }

private:
  std::string                                  mArgs;
  std::unordered_map<std::string, double>      numbers;
  std::unordered_map<std::string, std::string> strings;
};

int rf_limesuiteng_open_multi(char* args, void** h, uint32_t num_requested_channels)
{
  // Create handler
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)malloc(sizeof(rf_limesuiteng_handler_t));

  LimePluginContext* lime = new LimePluginContext();
  // lime->currentWorkingDirectory = std::string(hostState->path);
  lime->samplesFormat = DataFormat::F32;

  srsRAN_ParamProvider configProvider(args);

  // gathers arguments and initializes devices
  if (LimePlugin_Init(lime, LogCallback, &configProvider) != 0)
    return SRSRAN_ERROR;

  // runtime decided arguments
  LimeRuntimeParameters state;

  int rxCount = num_requested_channels;
  int txCount = num_requested_channels;

  for (int ch = 0; ch < rxCount; ++ch) {
    state.rx.freq.push_back(1e9);
    state.rx.gain.push_back(0);
    state.rx.bandwidth.push_back(20e6);
  }

  for (int ch = 0; ch < txCount; ++ch) {
    state.tx.freq.push_back(1e9);
    state.tx.gain.push_back(0);
    state.tx.bandwidth.push_back(20e6);
  }

  double sample_rate = 20e6;
  state.rf_ports.push_back({sample_rate, rxCount, txCount});

  int status = LimePlugin_Setup(lime, &state);
  if (status != 0)
    return SRSRAN_ERROR;

  return SRSRAN_SUCCESS;
}

int rf_limesuiteng_open(char* args, void** h)
{
  return rf_limesuiteng_open_multi(args, h, 1);
}

int rf_limesuiteng_close(void* h)
{
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  LimePluginContext*        lime    = handler->context;
  int                       status  = LimePlugin_Destroy(lime);
  free(handler);
  return status == 0 ? SRSRAN_SUCCESS : SRSRAN_ERROR;
}

static double rf_limesuiteng_set_srate(void* h, double rate, lime::TRXDir dir)
{
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  LimePluginContext*        lime    = handler->context;

  int      channelIndex = 0;
  int      oversample   = 0;
  int      chipIndex    = lime->rfdev[0].chipIndex;
  OpStatus status       = lime->rfdev[0].device->SetSampleRate(chipIndex, dir, 0, rate, oversample);
  if (status != OpStatus::Success)
    return SRSRAN_ERROR;

  DirectionalSettings& settings = dir == TRXDir::Rx ? lime->rfdev[0].configInputs.rx : lime->rfdev[0].configInputs.tx;

  status = lime->rfdev[0].device->SetLowPassFilter(
      chipIndex, dir, channelIndex, rate * lime->rfdev[0].configInputs.lpfBandwidthScale);
  if (status != OpStatus::Success)
    return SRSRAN_ERROR;

  ChannelConfig::Direction::GFIRFilter gfir;
  gfir.enabled   = settings.gfir_enable;
  gfir.bandwidth = rate;
  status         = lime->rfdev[0].device->ConfigureGFIR(chipIndex, dir, channelIndex, gfir);
  if (status != OpStatus::Success)
    return SRSRAN_ERROR;

  if (dir == TRXDir::Rx)
    handler->rx_rate = rate;
  else
    handler->tx_rate = rate;

  return rate;
}

double rf_limesuiteng_set_rx_srate(void* h, double rate)
{
  return rf_limesuiteng_set_srate(h, rate, TRXDir::Rx);
}

double rf_limesuiteng_set_tx_srate(void* h, double rate)
{
  return rf_limesuiteng_set_srate(h, rate, TRXDir::Tx);
}

int rf_limesuiteng_set_rx_gain(void* h, double gain)
{
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  LimePluginContext*        lime    = handler->context;
  for (size_t ch = 0; ch < lime->rxChannels.size(); ++ch) {
    if (rf_limesuiteng_set_rx_gain_ch(h, ch, gain) != SRSRAN_SUCCESS)
      return SRSRAN_ERROR;
  }
  return SRSRAN_SUCCESS;
}

int rf_limesuiteng_set_rx_gain_ch(void* h, uint32_t ch, double gain)
{
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  LimePluginContext*        lime    = handler->context;
  LimePlugin_SetRxGain(lime, gain, ch);
  return SRSRAN_SUCCESS;
}

int rf_limesuiteng_set_tx_gain(void* h, double gain)
{
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  LimePluginContext*        lime    = handler->context;
  for (size_t ch = 0; ch < lime->txChannels.size(); ++ch) {
    if (rf_limesuiteng_set_tx_gain_ch(h, ch, gain) != SRSRAN_SUCCESS)
      return SRSRAN_ERROR;
  }
  return SRSRAN_SUCCESS;
}

int rf_limesuiteng_set_tx_gain_ch(void* h, uint32_t ch, double gain)
{
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  LimePluginContext*        lime    = handler->context;
  LimePlugin_SetTxGain(lime, gain, ch);
  return SRSRAN_SUCCESS;
}

double rf_limesuiteng_get_rx_gain(void* h)
{
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  LimePluginContext*        lime    = handler->context;
  // unsigned           gain    = 0;
  // if (LMS_GetGaindB(handler->device, false, 0, &gain) != 0) {
  //   printf("LMS_GetGaindB: Failed get gain\n");
  //   return SRSRAN_ERROR;
  // }
  // return (double)gain;
  return SRSRAN_ERROR;
}

double rf_limesuiteng_get_tx_gain(void* h)
{
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  // unsigned           gain    = 0;
  // if (LMS_GetGaindB(handler->device, true, 0, &gain) != 0) {
  //   printf("LMS_GetGaindB: Failed get gain\n");
  //   return SRSRAN_ERROR;
  // }
  // return (double)gain;
  return SRSRAN_ERROR;
}

static srsran_rf_info_t gGainInfo = {0, 51, 0, 51};
srsran_rf_info_t*       rf_limesuiteng_get_info(void* h)
{
  return &gGainInfo;
}

static double rf_limesuiteng_set_freq(void* h, uint32_t ch, double freq, TRXDir dir)
{
  rf_limesuiteng_handler_t* handler     = (rf_limesuiteng_handler_t*)h;
  LimePluginContext*        lime        = handler->context;
  auto&                     channels    = dir == TRXDir::Rx ? lime->rxChannels : lime->txChannels;
  int                       chipChannel = channels.at(ch).chipChannel;
  int                       chipIndex   = channels.at(ch).parent->chipIndex;
  lime::OpStatus            status = channels.at(ch).parent->device->SetFrequency(chipIndex, dir, chipChannel, freq);
  if (status != OpStatus::Success)
    return SRSRAN_ERROR;
  return freq;
}

double rf_limesuiteng_set_rx_freq(void* h, uint32_t ch, double freq)
{
  return rf_limesuiteng_set_freq(h, ch, freq, TRXDir::Rx);
}

double rf_limesuiteng_set_tx_freq(void* h, uint32_t ch, double freq)
{
  return rf_limesuiteng_set_freq(h, ch, freq, TRXDir::Tx);
}

static void timestamp_to_secs(double rate, uint64_t timestamp, time_t* secs, double* frac_secs)
{
  double totalsecs = (double)timestamp / rate;
  if (secs && frac_secs) {
    *secs      = (time_t)totalsecs;
    *frac_secs = totalsecs - (*secs);
  }
}

void rf_limesuiteng_get_time(void* h, time_t* secs, double* frac_secs)
{
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  // TODO:
  // lms_stream_status_t status;
  // if (handler->rx_stream_active) {
  //   LMS_GetStreamStatus(&handler->rxStream[0], &status);
  //   timestamp_to_secs(handler->rx_rate, status.timestamp, secs, frac_secs);
  // }
}

int rf_limesuiteng_recv_with_time_multi(void*    h,
                                        void**   data,
                                        uint32_t nsamples,
                                        bool     blocking,
                                        time_t*  secs,
                                        double*  frac_secs)
{
  rf_limesuiteng_handler_t* handler                = (rf_limesuiteng_handler_t*)h;
  LimePluginContext*        lime                   = handler->context;
  lime::complex32f_t*       dest[SRSRAN_MAX_PORTS] = {0};

  for (size_t ch = 0; ch < lime->rxChannels.size(); ++ch) {
    cf_t* data_c = (cf_t*)data[ch];
    dest[ch]     = (lime::complex32f_t*)data_c;
  }

  lime::StreamMeta meta;
  int              samplesGot = LimePlugin_Read_complex32f(lime, dest, nsamples, 0, meta);
  if (samplesGot < 0)
    return SRSRAN_ERROR;

  if (secs != NULL && frac_secs != NULL)
    timestamp_to_secs(handler->rx_rate, meta.timestamp, secs, frac_secs);

  return samplesGot;
}

int rf_limesuiteng_recv_with_time(void*    h,
                                  void*    data,
                                  uint32_t nsamples,
                                  bool     blocking,
                                  time_t*  secs,
                                  double*  frac_secs)
{
  return rf_limesuiteng_recv_with_time_multi(h, &data, nsamples, blocking, secs, frac_secs);
}

int rf_limesuiteng_send_timed(void*  h,
                              void*  data,
                              int    nsamples,
                              time_t secs,
                              double frac_secs,
                              bool   has_time_spec,
                              bool   blocking,
                              bool   is_start_of_burst,
                              bool   is_end_of_burst)
{
  return rf_limesuiteng_send_timed_multi(
      h, &data, nsamples, secs, frac_secs, has_time_spec, blocking, is_start_of_burst, is_end_of_burst);
}

int rf_limesuiteng_send_timed_multi(void*  h,
                                    void*  data[SRSRAN_MAX_PORTS],
                                    int    nsamples,
                                    time_t secs,
                                    double frac_secs,
                                    bool   has_time_spec,
                                    bool   blocking,
                                    bool   is_start_of_burst,
                                    bool   is_end_of_burst)
{
  rf_limesuiteng_handler_t* handler = (rf_limesuiteng_handler_t*)h;
  LimePluginContext*        lime    = handler->context;

  StreamMeta meta{};
  meta.timestamp          = 0;
  meta.waitForTimestamp   = has_time_spec;
  meta.flushPartialPacket = is_end_of_burst;

  if (has_time_spec) {
    srsran_timestamp_t time = {secs, frac_secs};
    meta.timestamp          = srsran_timestamp_uint64(&time, handler->tx_rate);
  }

  lime::complex32f_t* src[SRSRAN_MAX_CHANNELS] = {};
  for (size_t ch = 0; ch < lime->txChannels.size(); ++ch)
    src[ch] = (lime::complex32f_t*)data[ch];

  int samplesSent =
      LimePlugin_Write_complex32f(lime, reinterpret_cast<const lime::complex32f_t* const*>(src), nsamples, 0, meta);
  if (samplesSent < 0)
    return SRSRAN_ERROR;

  return samplesSent;
}

rf_dev_t srsran_rf_dev_limesuiteng = {"limesuiteng",
                                      rf_limesuiteng_devname,
                                      rf_limesuiteng_start_rx_stream,
                                      rf_limesuiteng_stop_rx_stream,
                                      rf_limesuiteng_flush_buffer,
                                      rf_limesuiteng_has_rssi,
                                      rf_limesuiteng_get_rssi,
                                      rf_limesuiteng_suppress_stdout,
                                      rf_limesuiteng_register_error_handler,
                                      rf_limesuiteng_open,
                                      rf_limesuiteng_open_multi,
                                      rf_limesuiteng_close,
                                      rf_limesuiteng_set_rx_srate,
                                      rf_limesuiteng_set_rx_gain,
                                      rf_limesuiteng_set_rx_gain_ch,
                                      rf_limesuiteng_set_tx_gain,
                                      rf_limesuiteng_set_tx_gain_ch,
                                      rf_limesuiteng_get_tx_gain,
                                      rf_limesuiteng_get_rx_gain,
                                      rf_limesuiteng_get_info,
                                      rf_limesuiteng_set_rx_freq,
                                      rf_limesuiteng_set_tx_srate,
                                      rf_limesuiteng_set_tx_freq,
                                      rf_limesuiteng_get_time,
                                      NULL,
                                      rf_limesuiteng_recv_with_time,
                                      rf_limesuiteng_recv_with_time_multi,
                                      rf_limesuiteng_send_timed,
                                      .srsran_rf_send_timed_multi = rf_limesuiteng_send_timed_multi};

#ifdef ENABLE_RF_PLUGINS
int register_plugin(rf_dev_t** rf_api)
{
  if (rf_api == NULL) {
    return SRSRAN_ERROR;
  }
  *rf_api = &srsran_rf_dev_limesuiteng;
  return SRSRAN_SUCCESS;
}
#endif /* ENABLE_RF_PLUGINS */
