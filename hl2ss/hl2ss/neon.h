
#pragma once

#include "types.h"

void Neon_ZHTToNV12(u16 const* pDepth, u16 const* pAb, u8* pNV12);
void Neon_ZLTToBGRA8(u8 const* pSigma, u16 const* pDepth, u16 const* pAb, u32* pBGRA8);
