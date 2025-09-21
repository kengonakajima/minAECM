#ifndef AECM_H_
#define AECM_H_

#include <stddef.h>
#include <stdint.h>

#include "aecm_defines.h"


void InitAecm();
int ProcessBlock(const int16_t* farend,
                 const int16_t* nearend,
                 int16_t* out);

// デバッグ向け制御（0:有効, 非0:バイパス）。
void SetBypassSupMask(int enable);
void SetBypassNlp(int enable);

#endif  // AECM_H_
