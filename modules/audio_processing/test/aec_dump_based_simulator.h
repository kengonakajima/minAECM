/*
 *  Copyright (c) 2016 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#ifndef MODULES_AUDIO_PROCESSING_TEST_AEC_DUMP_BASED_SIMULATOR_H_
#define MODULES_AUDIO_PROCESSING_TEST_AEC_DUMP_BASED_SIMULATOR_H_

#include <fstream>
#include <string>

#include "modules/audio_processing/test/audio_processing_simulator.h"
#include "rtc_base/ignore_wundef.h"

RTC_PUSH_IGNORING_WUNDEF()
#ifdef WEBRTC_ANDROID_PLATFORM_BUILD
#include "external/webrtc/webrtc/modules/audio_processing/debug.pb.h"
#else
#include "modules/audio_processing/debug.pb.h"
#endif
RTC_POP_IGNORING_WUNDEF()

namespace web_rtc {
namespace test {

// Used to perform an audio processing simulation from an aec dump.
class AecDumpBasedSimulator final : public AudioProcessingSimulator {
 public:
  AecDumpBasedSimulator(const SimulationSettings& settings,
                        rtc::scoped_refptr<AudioProcessing> audio_processing,
                        std::unique_ptr<AudioProcessingBuilder> ap_builder);

  AecDumpBasedSimulator() = delete;
  AecDumpBasedSimulator(const AecDumpBasedSimulator&) = delete;
  AecDumpBasedSimulator& operator=(const AecDumpBasedSimulator&) = delete;

  ~AecDumpBasedSimulator() override;

  // Processes the messages in the aecdump file.
  void Process() override;

  // Analyzes the data in the aecdump file and reports the resulting statistics.
  void Analyze() override;

 private:
  void HandleEvent(const web_rtc::audioproc::Event& event_msg,
                   int& num_forward_chunks_processed,
                   int& init_index);
  void HandleMessage(const web_rtc::audioproc::Init& msg, int init_index);
  void HandleMessage(const web_rtc::audioproc::Stream& msg);
  void HandleMessage(const web_rtc::audioproc::ReverseStream& msg);
  void HandleMessage(const web_rtc::audioproc::Config& msg);
  void HandleMessage(const web_rtc::audioproc::RuntimeSetting& msg);
  void PrepareProcessStreamCall(const web_rtc::audioproc::Stream& msg);
  void PrepareReverseProcessStreamCall(
      const web_rtc::audioproc::ReverseStream& msg);
  void VerifyProcessStreamBitExactness(const web_rtc::audioproc::Stream& msg);
  void MaybeOpenCallOrderFile();
  enum InterfaceType {
    kFixedInterface,
    kFloatInterface,
    kNotSpecified,
  };

  FILE* dump_input_file_;
  std::unique_ptr<ChannelBuffer<float>> artificial_nearend_buf_;
  std::unique_ptr<ChannelBufferWavReader> artificial_nearend_buffer_reader_;
  bool artificial_nearend_eof_reported_ = false;
  InterfaceType interface_used_ = InterfaceType::kNotSpecified;
  std::unique_ptr<std::ofstream> call_order_output_file_;
  bool finished_processing_specified_init_block_ = false;
};

}  // namespace test
}  // namespace web_rtc

#endif  // MODULES_AUDIO_PROCESSING_TEST_AEC_DUMP_BASED_SIMULATOR_H_
