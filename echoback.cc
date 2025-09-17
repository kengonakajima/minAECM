// Echoback (C++): 最小構成のローカル・エコーバック + AECM
// Usage:
//   ./echoback [--passthrough]
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

void process_available_blocks(State& s){
  // Run as many 64-sample blocks as possible
  while (s.rec_dev.size() >= (size_t)AECM_BLOCK_SIZE) {
    std::vector<int16_t> near_blk(AECM_BLOCK_SIZE), far_blk(AECM_BLOCK_SIZE), out_blk(AECM_BLOCK_SIZE);
    // 1) pop near block
    pop_samples(s.rec_dev, near_blk.data(), AECM_BLOCK_SIZE);
    // 2) pop far block（スピーカへ送る信号 = 参照）
    if (s.jitter.size() >= (size_t)AECM_BLOCK_SIZE) {
      pop_samples(s.jitter, far_blk.data(), AECM_BLOCK_SIZE);
    } else {
      std::memset(far_blk.data(), 0, AECM_BLOCK_SIZE * sizeof(int16_t));
    }

    if (s.passthrough) {
      std::memcpy(out_blk.data(), near_blk.data(), AECM_BLOCK_SIZE * sizeof(int16_t));
    } else {
      // AECM: 先に far をバッファリングし、その後 near を処理
      BufferFarend(far_blk.data());
      Process(near_blk.data(),
                   out_blk.data());
    }

    // ローカル・ループバック: 処理後の出力を蓄積して、次々回以降の far にする
    push_block(s.jitter, out_blk.data(), AECM_BLOCK_SIZE);

    // 今回のスピーカ出力は `far_blk`
    push_block(s.out_dev, far_blk.data(), AECM_BLOCK_SIZE);
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
  if (in) { for (unsigned long i=0;i<n;i++) st->rec_dev.push_back(in[i]); }

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

  // 引数パース
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i] ? argv[i] : "");
    if (arg == "--passthrough" || arg == "-p") {
      s.passthrough = true;
    } else if (arg == "--no-wiener" || arg == "--no-suppress") {
      s.bypass_wiener = true;
    } else if (arg == "--no-nlp") {
      s.bypass_nlp = true;
    } else if (arg == "--help" || arg == "-h") {
      std::fprintf(stderr,
                   "Usage: %s [--passthrough] [--no-wiener|--no-suppress] [--no-nlp]\n",
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

  err = Pa_OpenStream(&stream, &inP, &outP, AECM_SAMPLE_RATE_HZ, AECM_BLOCK_SIZE, paClipOff, pa_callback, &s);
  if (err!=paNoError){ std::fprintf(stderr, "Pa_OpenStream error %s\n", Pa_GetErrorText(err)); Pa_Terminate(); return 1; }
  err = Pa_StartStream(stream);
  if (err!=paNoError){ std::fprintf(stderr, "Pa_StartStream error %s\n", Pa_GetErrorText(err)); Pa_CloseStream(stream); Pa_Terminate(); return 1; }

  std::fprintf(stderr, "Running... Ctrl-C to stop.\n");
  // AECM 初期化
  if (!s.passthrough) {
    if (Init() != 0) {
      std::fprintf(stderr, "AECM init failed\n");
      Pa_StopStream(stream); Pa_CloseStream(stream); Pa_Terminate(); return 1;
    }
    SetBypassWiener(s.bypass_wiener ? 1 : 0);
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
