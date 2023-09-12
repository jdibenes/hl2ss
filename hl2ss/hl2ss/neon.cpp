
#include <Windows.h>
#include "research_mode.h"
#include "types.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Neon_ZHTInvalidate(u16 const* pDepth, u16 *pOut)
{
	uint16x8_t threshold = vdupq_n_u16(RM_ZHT_MASK);

	for (int i = 0; i < (RM_ZHT_PIXELS / 32); ++i)
	{
	uint16x8x4_t d = vld1q_u16_x4(pDepth);

	d.val[0] = vandq_u16(d.val[0], vcltq_u16(d.val[0], threshold));
	d.val[1] = vandq_u16(d.val[1], vcltq_u16(d.val[1], threshold));
	d.val[2] = vandq_u16(d.val[2], vcltq_u16(d.val[2], threshold));
	d.val[3] = vandq_u16(d.val[3], vcltq_u16(d.val[3], threshold));

	vst1q_u16_x4(pOut, d);

	pDepth += 32;
	pOut   += 32;
	}
}

// OK
void Neon_XABToNV12(u16 const* pAb, u8* pOut)
{
	for (int i = 0; i < (RM_ZHT_PIXELS / 32); ++i)
	{
	uint16x8x4_t d = vld1q_u16_x4(pAb);
	uint8x8x4_t  b;

	b.val[0] = vmovn_u16(vcvtq_u16_f16(vsqrtq_f16(vcvtq_f16_u16(d.val[0]))));
	b.val[1] = vmovn_u16(vcvtq_u16_f16(vsqrtq_f16(vcvtq_f16_u16(d.val[1]))));
	b.val[2] = vmovn_u16(vcvtq_u16_f16(vsqrtq_f16(vcvtq_f16_u16(d.val[2]))));
	b.val[3] = vmovn_u16(vcvtq_u16_f16(vsqrtq_f16(vcvtq_f16_u16(d.val[3]))));

	vst1_u8_x4(pOut, b);

	pAb  += 32;
	pOut += 32;
	}

	memset(pOut, NV12_ZERO_CHROMA, RM_ZHT_PIXELS / 2);
}

// OK
void Neon_ZABToNV12(u16 const* pDepth, u16 const* pAb, u8* pNV12)
{
	uint16x8_t threshold = vdupq_n_u16(RM_ZHT_MASK);

	for (int i = 0; i < (RM_ZHT_PIXELS / 32); ++i)
	{
	uint16x8x4_t d = vld1q_u16_x4(pDepth);
	uint8x8x4_t  b;

	b.val[0] = vshrn_n_u16(vandq_u16(d.val[0], vcltq_u16(d.val[0], threshold)), 2);
	b.val[1] = vshrn_n_u16(vandq_u16(d.val[1], vcltq_u16(d.val[1], threshold)), 2);
	b.val[2] = vshrn_n_u16(vandq_u16(d.val[2], vcltq_u16(d.val[2], threshold)), 2);
	b.val[3] = vshrn_n_u16(vandq_u16(d.val[3], vcltq_u16(d.val[3], threshold)), 2);

	vst1_u8_x4(pNV12, b);

	pDepth += 32;
	pNV12  += 32;
	}
	
	for (int i = 0; i < (RM_ZHT_PIXELS / 16); ++i)
	{
	uint16x8x2_t d = vld2q_u16(pAb);
	uint8x8_t    b = vmovn_u16(vcvtq_u16_f16(vsqrtq_f16(vcvtq_f16_u16(d.val[0]))));

	vst1_u8(pNV12, b);

	pAb    += 16;
	pNV12  += 8;
	}
}

// OK
void Neon_ZLTToBGRA8(u8 const* pSigma, u16 const* pDepth, u16 const* pAb, u32* pBGRA8)
{
	uint16x8_t threshold = vdupq_n_u16(RM_ZLT_MASK);

	for (int i = 0; i < (RM_ZLT_PIXELS / 32); ++i)
	{
	uint8x8x4_t  s = vld1_u8_x4(pSigma);
	uint16x8x4_t d = vld1q_u16_x4(pDepth);

	d.val[0] = vandq_u16(d.val[0], vcltq_u16(vmovl_u8(s.val[0]), threshold));
	d.val[1] = vandq_u16(d.val[1], vcltq_u16(vmovl_u8(s.val[1]), threshold));
	d.val[2] = vandq_u16(d.val[2], vcltq_u16(vmovl_u8(s.val[2]), threshold));
	d.val[3] = vandq_u16(d.val[3], vcltq_u16(vmovl_u8(s.val[3]), threshold));

	vst1q_u16_x4(pBGRA8, d);

	pSigma += 32;
	pDepth += 32;
	pBGRA8 += 16;
	}

	memcpy(pBGRA8, pAb, RM_ZLT_ABSIZE);
}
