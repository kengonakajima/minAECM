#ifndef AECM_H_
#define AECM_H_

#include <stddef.h>
#include <stdint.h>

#include "aecm_defines.h"

// エラーコード
#define AECM_UNSPECIFIED_ERROR 12000
#define AECM_UNSUPPORTED_FUNCTION_ERROR 12001
#define AECM_UNINITIALIZED_ERROR 12002
#define AECM_NULL_POINTER_ERROR 12003
#define AECM_BAD_PARAMETER_ERROR 12004

// ワーニングコード
#define AECM_BAD_PARAMETER_WARNING 12100

// AECM の単一インスタンス API。
// Init/BufferFarend/Process は 16kHz, FRAME_LEN (=64) 固定前提。
int32_t Init();
int32_t BufferFarend(const int16_t* farend);
int32_t Process(const int16_t* nearend, int16_t* out);

// デバッグ向け制御（0:有効, 非0:バイパス）。
void SetBypassWiener(int enable);
void SetBypassNlp(int enable);

#endif  // AECM_H_
