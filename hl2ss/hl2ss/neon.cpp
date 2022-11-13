
#include <Windows.h>
#include "types.h"

void Neon_DepthAHATToNV12(u16 const* pDepth, u16 const* pAb, u8* pNV12)
{
	uint16x8_t threshold = vdupq_n_u16(4090);

	for (int i = 0; i < ((512 * 512) / 32); ++i)
	{
	uint16x8x4_t d = vld1q_u16_x4(pDepth);
	uint8x8x4_t  b;

	b.val[0] = vshrn_n_u16(vandq_u16(d.val[0], vcltq_u16(d.val[0], threshold)), 2);
	b.val[1] = vshrn_n_u16(vandq_u16(d.val[1], vcltq_u16(d.val[1], threshold)), 2);
	b.val[2] = vshrn_n_u16(vandq_u16(d.val[2], vcltq_u16(d.val[2], threshold)), 2);
	b.val[3] = vshrn_n_u16(vandq_u16(d.val[3], vcltq_u16(d.val[3], threshold)), 2);

	vst1_u8_x4(pNV12, b);

	pDepth +=  32;
	pNV12  +=  32;
	}
	
	for (int j = 0; j < 512; ++j)
	{
	for (int i = 0; i < (512 / 32); ++i)
	{
	uint16x8x4_t d = vld1q_u16_x4(pAb);
	uint8x8x2_t  b;

	b.val[0] = vshrn_n_s16(d.val[0], 8);
	b.val[1] = vshrn_n_s16(d.val[2], 8);

	vst1_u8_x2(pNV12, b);

	pAb    +=  32;
	pNV12  +=  16;
	}
	}
}
