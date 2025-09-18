#ifndef AECM_H_
#define AECM_H_

#include <stddef.h>
#include <stdint.h>

#include "aecm_defines.h"


int32_t Init();
int32_t Process(const int16_t* farend,
                const int16_t* nearend,
                int16_t* out);

// デバッグ向け制御（0:有効, 非0:バイパス）。
void SetBypassWiener(int enable);
void SetBypassNlp(int enable);

#endif  // AECM_H_
