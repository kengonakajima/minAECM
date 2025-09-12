/*
 *  Copyright (c) 2012 The WebRTC project authors. All Rights Reserved.
 *
 *  Use of this source code is governed by a BSD-style license
 *  that can be found in the LICENSE file in the root of the source
 *  tree. An additional intellectual property rights grant can be found
 *  in the file PATENTS.  All contributing project authors may
 *  be found in the AUTHORS file in the root of the source tree.
 */

#include <stdio.h>

#include <string>
#include <vector>

#include "modules/audio_coding/include/audio_coding_module.h"
#include "modules/audio_coding/test/EncodeDecodeTest.h"
#include "modules/audio_coding/test/PacketLossTest.h"
#include "modules/audio_coding/test/TestAllCodecs.h"
#include "modules/audio_coding/test/TestRedFec.h"
#include "modules/audio_coding/test/TestStereo.h"
#include "modules/audio_coding/test/TestVADDTX.h"
#include "modules/audio_coding/test/TwoWayCommunication.h"
#include "modules/audio_coding/test/iSACTest.h"
#include "modules/audio_coding/test/opus_test.h"
#include "test/gtest.h"
#include "test/testsupport/file_utils.h"

TEST(AudioCodingModuleTest, TestAllCodecs) {
  web_rtc::TestAllCodecs().Perform();
}

#if defined(WEBRTC_ANDROID)
TEST(AudioCodingModuleTest, DISABLED_TestEncodeDecode) {
#else
TEST(AudioCodingModuleTest, TestEncodeDecode) {
#endif
  web_rtc::EncodeDecodeTest().Perform();
}

TEST(AudioCodingModuleTest, TestRedFec) {
  web_rtc::TestRedFec().Perform();
}

#if defined(WEBRTC_ANDROID)
TEST(AudioCodingModuleTest, DISABLED_TestIsac) {
#else
TEST(AudioCodingModuleTest, TestIsac) {
#endif
  web_rtc::ISACTest().Perform();
}

#if (defined(WEBRTC_CODEC_ISAC) || defined(WEBRTC_CODEC_ISACFX)) && \
    defined(WEBRTC_CODEC_ILBC)
#if defined(WEBRTC_ANDROID)
TEST(AudioCodingModuleTest, DISABLED_TwoWayCommunication) {
#else
TEST(AudioCodingModuleTest, TwoWayCommunication) {
#endif
  web_rtc::TwoWayCommunication().Perform();
}
#endif

// Disabled on ios as flaky, see https://crbug.com/webrtc/7057
#if defined(WEBRTC_ANDROID) || defined(WEBRTC_IOS)
TEST(AudioCodingModuleTest, DISABLED_TestStereo) {
#else
TEST(AudioCodingModuleTest, TestStereo) {
#endif
  web_rtc::TestStereo().Perform();
}

TEST(AudioCodingModuleTest, TestWebRtcVadDtx) {
  web_rtc::TestWebRtcVadDtx().Perform();
}

TEST(AudioCodingModuleTest, TestOpusDtx) {
  web_rtc::TestOpusDtx().Perform();
}

// Disabled on ios as flaky, see https://crbug.com/webrtc/7057
#if defined(WEBRTC_IOS)
TEST(AudioCodingModuleTest, DISABLED_TestOpus) {
#else
TEST(AudioCodingModuleTest, TestOpus) {
#endif
  web_rtc::OpusTest().Perform();
}

TEST(AudioCodingModuleTest, TestPacketLoss) {
  web_rtc::PacketLossTest(1, 10, 10, 1).Perform();
}

TEST(AudioCodingModuleTest, TestPacketLossBurst) {
  web_rtc::PacketLossTest(1, 10, 10, 2).Perform();
}

// Disabled on ios as flake, see https://crbug.com/webrtc/7057
#if defined(WEBRTC_IOS)
TEST(AudioCodingModuleTest, DISABLED_TestPacketLossStereo) {
#else
TEST(AudioCodingModuleTest, TestPacketLossStereo) {
#endif
  web_rtc::PacketLossTest(2, 10, 10, 1).Perform();
}

// Disabled on ios as flake, see https://crbug.com/webrtc/7057
#if defined(WEBRTC_IOS)
TEST(AudioCodingModuleTest, DISABLED_TestPacketLossStereoBurst) {
#else
TEST(AudioCodingModuleTest, TestPacketLossStereoBurst) {
#endif
  web_rtc::PacketLossTest(2, 10, 10, 2).Perform();
}

// The full API test is too long to run automatically on bots, but can be used
// for offline testing. User interaction is needed.
#ifdef ACM_TEST_FULL_API
TEST(AudioCodingModuleTest, TestAPI) {
  web_rtc::APITest().Perform();
}
#endif
