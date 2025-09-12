// Offline comparator: feed two WAVs (render x, capture y) into AECM and write processed.wav
#include <cstdio>
#include <cstdint>
#include <vector>
#include <string>
#include <fstream>
#include <cstring>

#include "modules/audio_processing/aecm/echo_control_mobile.h"

struct Wav {
  int sr = 0;
  int ch = 0;
  std::vector<int16_t> samples;
};

static uint32_t rd32le(const uint8_t* p){ return p[0] | (p[1]<<8) | (p[2]<<16) | (p[3]<<24); }
static uint16_t rd16le(const uint8_t* p){ return p[0] | (p[1]<<8); }

static bool read_wav_pcm16(const std::string& path, Wav* out){
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::vector<uint8_t> buf((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  if (buf.size() < 44) return false;
  if (std::memcmp(buf.data(), "RIFF",4) || std::memcmp(buf.data()+8,"WAVE",4)) return false;
  size_t pos = 12; int sr=0,ch=0,bps=0; size_t data_off=0,data_size=0;
  while (pos + 8 <= buf.size()){
    uint32_t id = rd32le(&buf[pos]); pos+=4; uint32_t sz = rd32le(&buf[pos]); pos+=4; size_t start=pos;
    if (id == 0x20746d66){ // 'fmt '
      uint16_t fmt = rd16le(&buf[start+0]); ch = rd16le(&buf[start+2]); sr = rd32le(&buf[start+4]); bps = rd16le(&buf[start+14]);
      if (fmt != 1 || bps != 16) return false;
    } else if (id == 0x61746164){ // 'data'
      data_off = start; data_size = sz; break;
    }
    pos = start + sz;
  }
  if (!data_off || !data_size) return false;
  out->sr = sr; out->ch = ch; size_t ns = data_size/2; out->samples.resize(ns);
  const int16_t* p = reinterpret_cast<const int16_t*>(&buf[data_off]);
  for (size_t i=0;i<ns;i++) out->samples[i] = p[i];
  return true;
}

int main(int argc, char** argv){
  if (argc < 3){ std::fprintf(stderr, "Usage: %s <render.wav> <capture.wav>\n", argv[0]); return 1; }
  Wav x, y;
  if (!read_wav_pcm16(argv[1], &x) || !read_wav_pcm16(argv[2], &y)){ std::fprintf(stderr, "Failed to read wavs\n"); return 1; }
  if (x.sr!=16000 || y.sr!=16000 || x.ch!=1 || y.ch!=1){ std::fprintf(stderr, "Expected 16k mono wavs\n"); }
  const size_t kBlockSize = 160; // 10ms @16kHz
  size_t N = std::min(x.samples.size(), y.samples.size()) / kBlockSize;
  void* aecm = web_rtc::WebRtcAecm_Create();
  if (!aecm){ std::fprintf(stderr, "AECM create failed\n"); return 1; }
  if (web_rtc::WebRtcAecm_Init(aecm, 16000) != 0){ std::fprintf(stderr, "AECM init failed\n"); web_rtc::WebRtcAecm_Free(aecm); return 1; }
  web_rtc::AecmConfig cfg; cfg.cngMode = web_rtc::AecmTrue; cfg.echoMode = 3; web_rtc::WebRtcAecm_set_config(aecm, cfg);
  std::vector<int16_t> processed;
  processed.resize(N * kBlockSize);
  for (size_t n=0;n<N;n++){
    // Farend/render
    web_rtc::WebRtcAecm_BufferFarend(aecm, &x.samples[n*kBlockSize], kBlockSize);
    // Nearend/capture -> processed
    web_rtc::WebRtcAecm_Process(aecm,
                                &y.samples[n*kBlockSize],
                                nullptr,
                                &processed[n*kBlockSize],
                                kBlockSize,
                                50 /*msInSndCardBuf*/);
  }
  web_rtc::WebRtcAecm_Free(aecm);
  // Save processed signal as processed.wav (PCM16 mono 16kHz)
  const uint32_t sr = 16000;
  const uint16_t ch = 1;
  const uint16_t bps = 16;
  const uint32_t data_bytes = static_cast<uint32_t>(processed.size() * sizeof(int16_t));
  const uint32_t byte_rate = sr * ch * (bps/8);
  const uint16_t block_align = ch * (bps/8);
  std::ofstream wf("processed.wav", std::ios::binary);
  if (wf){
    // RIFF header
    wf.write("RIFF",4);
    uint32_t file_size_minus_8 = 36 + data_bytes; wf.write(reinterpret_cast<const char*>(&file_size_minus_8),4);
    wf.write("WAVE",4);
    // fmt chunk
    wf.write("fmt ",4);
    uint32_t fmt_size = 16; wf.write(reinterpret_cast<const char*>(&fmt_size),4);
    uint16_t audio_format = 1; wf.write(reinterpret_cast<const char*>(&audio_format),2);
    wf.write(reinterpret_cast<const char*>(&ch),2);
    wf.write(reinterpret_cast<const char*>(&sr),4);
    wf.write(reinterpret_cast<const char*>(&byte_rate),4);
    wf.write(reinterpret_cast<const char*>(&block_align),2);
    wf.write(reinterpret_cast<const char*>(&bps),2);
    // data chunk
    wf.write("data",4);
    wf.write(reinterpret_cast<const char*>(&data_bytes),4);
    wf.write(reinterpret_cast<const char*>(processed.data()), data_bytes);
  }
  return 0;
}
