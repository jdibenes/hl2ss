
#pragma once

#include "types.h"

void Neon_ZHTInvalidate(u16 const* pDepth, u16* pOut);
void Neon_XABToNV12(u16 const* pAb, u8* pOut);
void Neon_ZABToNV12(u16 const* pDepth, u16 const* pAb, u8* pNV12);
void Neon_ZLTToBGRA8(u8 const* pSigma, u16 const* pDepth, u16 const* pAb, u32* pBGRA8);
void Neon_F32ToS16(float const* in, int32_t elements, s16* out);
void Neon_S16MonoToStereo(int16_t const* in, int32_t elements, int16_t* out);
void Neon_F32CropAudio11to5(float const* in, int32_t elements, float* out);
