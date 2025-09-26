// Echoback (C++): 最小構成のローカル・エコーバック + AECM
// Usage:
//   ./echoback [--passthrough] [--input-delay-ms <ms>] [--loopback-delay-ms <ms>]
// 前提:
//   - 16 kHz モノラル固定, 16-bit I/O (PortAudio デフォルトデバイス)
//   - AECM は 4ms（64サンプル）単位で処理
//   - 参照信号は直前にスピーカへ出したブロック（ローカル・ループバック相当）

#include <portaudio.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <vector>

constexpr long long kDefaultLoopbackDelayMs = 150;

#include "aecm.h"
#include "aecm_defines.h"

 

struct State {
  // 1ch 固定（PortAudioデバイス設定も1ch）
  // ジッタバッファは固定遅延（最小構成: 内部固定値）

  // FIFOs in device domain (dev_sr)
  std::deque<int16_t> rec_dev;     // mic captured samples
  std::deque<int16_t> out_dev;     // to speaker

  // AECM domain = device domain (same sr). 64-sample blocks

  // Jitter buffer (device domain). Local echo path accumulator, mixes into speaker
  std::deque<int16_t> jitter;      // accumulate processed to emulate loopback latency

  // Optional loopback delay (speaker path). FIFO pre-filled with zeros.
  size_t loopback_delay_target_samples = 0;
  std::deque<int16_t> loopback_delay_fifo;  // speaker path delay line

  // Optional startup delay for capture stream (educational jitter buffer)
  std::deque<int16_t> delay_line;  // raw capture samples waiting for release
  size_t delay_target_samples = 0; // requested delay in samples

  // --passthrough: AEC を行わず素通し再生
  bool passthrough = false;
  bool bypass_wiener = false;
  bool bypass_nlp = false;
  
  // 単一インスタンス化API。個別インスタンスは不要。
};

size_t pop_samples(std::deque<int16_t>& q, int16_t* dst, size_t n){
  size_t m = q.size()<n ? q.size() : n;
  for (size_t i=0;i<m;i++){ dst[i]=q.front(); q.pop_front(); }
  return m;
}

void push_block(std::deque<int16_t>& q, const int16_t* src, size_t n){
  for (size_t i=0;i<n;i++) q.push_back(src[i]);
}

size_t ms_to_aligned_samples(long long delay_ms) {
  if (delay_ms <= 0) {
    return 0;
  }
  const size_t block = static_cast<size_t>(BLOCK_LEN);
  const long long sr = static_cast<long long>(SAMPLE_RATE_HZ);
  long long raw_samples_ll = (delay_ms * sr + 999) / 1000;
  if (raw_samples_ll <= 0) {
    return 0;
  }
  size_t raw_samples = static_cast<size_t>(raw_samples_ll);
  size_t delay_blocks = (raw_samples + block / 2) / block;
  return delay_blocks * block;
}

inline void enqueue_capture_sample(State& s, int16_t sample) {
  s.delay_line.push_back(sample);
  if (s.delay_line.size() <= s.delay_target_samples) {
    s.rec_dev.push_back(0);
  } else {
    s.rec_dev.push_back(s.delay_line.front());
    s.delay_line.pop_front();
  }
}

void process_available_blocks(State& s){
  // Run as many 64-sample blocks as possible
  while (s.rec_dev.size() >= (size_t)BLOCK_LEN) {
    std::vector<int16_t> near_blk(BLOCK_LEN), far_blk(BLOCK_LEN), out_blk(BLOCK_LEN);
    std::vector<int16_t> speaker_blk;
    if (s.loopback_delay_target_samples > 0) {
      speaker_blk.resize(BLOCK_LEN);
    }
    // 1) pop near block
    pop_samples(s.rec_dev, near_blk.data(), BLOCK_LEN);
    // 2) pop far block（スピーカへ送る信号 = 参照）
    if (s.jitter.size() >= (size_t)BLOCK_LEN) {
      pop_samples(s.jitter, far_blk.data(), BLOCK_LEN);
    } else {
      std::memset(far_blk.data(), 0, BLOCK_LEN * sizeof(int16_t));
    }

    if (s.passthrough) {
      std::memcpy(out_blk.data(), near_blk.data(), BLOCK_LEN * sizeof(int16_t));
    } else {
      // AECM: Far/Near ブロックを同時に処理
      ProcessBlock(far_blk.data(),
                   near_blk.data(),
                   out_blk.data());
    }

    // ローカル・ループバック: 処理後の出力を蓄積して、次々回以降の far にする
    push_block(s.jitter, out_blk.data(), BLOCK_LEN);

    if (s.loopback_delay_target_samples > 0) {
      push_block(s.loopback_delay_fifo, far_blk.data(), BLOCK_LEN);
      if (s.loopback_delay_fifo.size() >= (size_t)BLOCK_LEN) {
        pop_samples(s.loopback_delay_fifo, speaker_blk.data(), BLOCK_LEN);
      } else {
        std::memset(speaker_blk.data(), 0, BLOCK_LEN * sizeof(int16_t));
      }
      push_block(s.out_dev, speaker_blk.data(), BLOCK_LEN);
    } else {
      // 今回のスピーカ出力は `far_blk`
      push_block(s.out_dev, far_blk.data(), BLOCK_LEN);
    }
  }
}

int pa_callback(const void* inputBuffer,
                void* outputBuffer,
                unsigned long blockSize,
                const PaStreamCallbackTimeInfo* /*timeInfo*/,
                PaStreamCallbackFlags /*statusFlags*/,
                void* userData){
  auto* st = reinterpret_cast<State*>(userData);
  const int16_t* in = reinterpret_cast<const int16_t*>(inputBuffer);
  int16_t* out = reinterpret_cast<int16_t*>(outputBuffer);
  const unsigned long n = blockSize; // mono (framesPerBuffer)
  // Single-threaded実行のため終了フラグは不要

  // 1) enqueue capture
  for (unsigned long i = 0; i < n; ++i) {
    int16_t sample = in ? in[i] : 0;
    enqueue_capture_sample(*st, sample);
  }

  // 2) run AEC3 on as many blocks as ready
  process_available_blocks(*st);

  // 3) emit output
  for (unsigned long i=0;i<n;i++){
    if (!st->out_dev.empty()){ out[i]=st->out_dev.front(); st->out_dev.pop_front(); }
    else { out[i]=0; }
  }
  return paContinue;
}

int main(int argc, char** argv){
  State s;
  // 16k/64サンプル固定

  s.loopback_delay_target_samples = ms_to_aligned_samples(kDefaultLoopbackDelayMs);

  // 引数パース
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i] ? argv[i] : "");
    if (arg == "--passthrough" || arg == "-p") {
      s.passthrough = true;
    } else if (arg == "--no-wiener" || arg == "--no-suppress") {
      s.bypass_wiener = true;
    } else if (arg == "--no-nlp") {
      s.bypass_nlp = true;
    } else if (arg.rfind("--input-delay-ms=", 0) == 0) {
      std::string value = arg.substr(strlen("--input-delay-ms="));
      long long delay_ms = std::stoll(value);
      if (delay_ms < 0) delay_ms = 0;
      s.delay_target_samples = ms_to_aligned_samples(delay_ms);
    } else if (arg == "--input-delay-ms" && i + 1 < argc) {
      std::string value(argv[++i] ? argv[i] : "0");
      long long delay_ms = std::stoll(value);
      if (delay_ms < 0) delay_ms = 0;
      s.delay_target_samples = ms_to_aligned_samples(delay_ms);
    } else if (arg.rfind("--loopback-delay-ms=", 0) == 0) {
      std::string value = arg.substr(strlen("--loopback-delay-ms="));
      long long delay_ms = std::stoll(value);
      if (delay_ms < 0) delay_ms = 0;
      s.loopback_delay_target_samples = ms_to_aligned_samples(delay_ms);
    } else if (arg == "--loopback-delay-ms" && i + 1 < argc) {
      std::string value(argv[++i] ? argv[i] : "0");
      long long delay_ms = std::stoll(value);
      if (delay_ms < 0) delay_ms = 0;
      s.loopback_delay_target_samples = ms_to_aligned_samples(delay_ms);
    } else if (arg == "--help" || arg == "-h") {
      std::fprintf(stderr,
                   "Usage: %s [--passthrough] [--no-wiener|--no-suppress] [--no-nlp] [--input-delay-ms <ms>] [--loopback-delay-ms <ms>]\n",
                   argv[0]);
      return 0;
    }
  }
  const char* mode = s.passthrough ? "passthrough" : "aecm";
  if (s.passthrough) {
    std::fprintf(stderr, "echoback (16k mono): mode=%s\n", mode);
  } else {
    std::fprintf(stderr,
                 "echoback (16k mono): mode=%s (wiener=%s, nlp=%s)\n",
                 mode,
                 s.bypass_wiener ? "off" : "on",
                 s.bypass_nlp ? "off" : "on");
  }
  // 固定設定のため追加初期化不要

  if (s.delay_target_samples > 0) {
    double delay_ms = static_cast<double>(s.delay_target_samples) * 1000.0 /
                      static_cast<double>(SAMPLE_RATE_HZ);
    double blocks = static_cast<double>(s.delay_target_samples) /
                    static_cast<double>(BLOCK_LEN);
    std::fprintf(stderr, "capture delay: %.1f ms (%.0f samples, %.1f blocks)\n",
                 delay_ms,
                 static_cast<double>(s.delay_target_samples),
                 blocks);
  }

  if (s.loopback_delay_target_samples > 0) {
    double delay_ms = static_cast<double>(s.loopback_delay_target_samples) * 1000.0 /
                      static_cast<double>(SAMPLE_RATE_HZ);
    double blocks = static_cast<double>(s.loopback_delay_target_samples) /
                    static_cast<double>(BLOCK_LEN);
    std::fprintf(stderr,
                 "loopback delay: %.1f ms (%.0f samples, %.1f blocks)\n",
                 delay_ms,
                 static_cast<double>(s.loopback_delay_target_samples),
                 blocks);
  }

  if (s.loopback_delay_target_samples > 0) {
    const size_t block = static_cast<size_t>(BLOCK_LEN);
    std::vector<int16_t> zeros(block, 0);
    size_t blocks = s.loopback_delay_target_samples / block;
    for (size_t b = 0; b < blocks; ++b) {
      push_block(s.loopback_delay_fifo, zeros.data(), block);
    }
  }

  PaError err = Pa_Initialize();
  if (err!=paNoError){ std::fprintf(stderr, "Pa_Initialize error %s\n", Pa_GetErrorText(err)); return 1; }

  PaStream* stream=nullptr; PaStreamParameters inP{}, outP{};
  inP.device = Pa_GetDefaultInputDevice();
  outP.device = Pa_GetDefaultOutputDevice();
  if (inP.device<0 || outP.device<0){ std::fprintf(stderr, "No default device.\n"); Pa_Terminate(); return 1; }
  inP.channelCount = 1; inP.sampleFormat = paInt16;
  inP.suggestedLatency = Pa_GetDeviceInfo(inP.device)->defaultLowInputLatency;
  outP.channelCount = 1; outP.sampleFormat = paInt16;
  outP.suggestedLatency = Pa_GetDeviceInfo(outP.device)->defaultLowOutputLatency;

  err = Pa_OpenStream(&stream, &inP, &outP, SAMPLE_RATE_HZ, BLOCK_LEN, paClipOff, pa_callback, &s);
  if (err!=paNoError){ std::fprintf(stderr, "Pa_OpenStream error %s\n", Pa_GetErrorText(err)); Pa_Terminate(); return 1; }
  err = Pa_StartStream(stream);
  if (err!=paNoError){ std::fprintf(stderr, "Pa_StartStream error %s\n", Pa_GetErrorText(err)); Pa_CloseStream(stream); Pa_Terminate(); return 1; }

  std::fprintf(stderr, "Running... Ctrl-C to stop.\n");
  // AECM 初期化
  if (!s.passthrough) {
    InitAecm();
    SetBypassSupMask(s.bypass_wiener ? 1 : 0);
    SetBypassNlp(s.bypass_nlp ? 1 : 0);
  }
  while (Pa_IsStreamActive(stream)==1) {
    Pa_Sleep(100);
  }
  Pa_StopStream(stream); Pa_CloseStream(stream);
  Pa_Terminate();
  std::fprintf(stderr, "stopped.\n");
  return 0;
}
