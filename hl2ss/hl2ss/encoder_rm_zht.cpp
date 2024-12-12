
#include "encoder_rm_zht.h"
#include "research_mode.h"
#include "timestamp.h"

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

RM_ZHT_VideoTransform const Encoder_RM_ZHT::m_vt_lut[4] =
{
	{BypassZ, SetZAB,        RM_ZHT_PIXELS * 4,      VideoSubtype::VideoSubtype_ARGB},
	{BypassZ, SetZABToNV12, (RM_ZHT_PIXELS * 3) / 2, VideoSubtype::VideoSubtype_NV12},
	{AppendZ, SetXAB,        RM_ZHT_PIXELS * 2,      VideoSubtype::VideoSubtype_L16},
	{AppendZ, SetXABToNV12, (RM_ZHT_PIXELS * 3) / 2, VideoSubtype::VideoSubtype_NV12}
};

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// Set invalid depths (depth >= RM_ZHT_MASK) to 0 
// OK
void Encoder_RM_ZHT::InvalidateZ(uint16_t const* pDepth, uint16_t *pOut)
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

// Store AB image in NV12 image
// Y  = (u8)(u16)sqrt((f16)AB)
// UV = no color
// OK
void Encoder_RM_ZHT::XABToNV12(uint16_t const* pAb, uint8_t* pOut)
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

// Store Z and AB images as a single NV12 image
// Y  = (u8)(invalidate(Z) >> 2)
// UV = (u8)(u16)sqrt((f16)subsample(AB))
// OK
void Encoder_RM_ZHT::ZABToNV12(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* pNV12)
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
void Encoder_RM_ZHT::SetZAB(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* out)
{
    InvalidateZ(pDepth, reinterpret_cast<uint16_t*>(out));
    memcpy(out + RM_ZHT_ZSIZE, pAb, RM_ZHT_ABSIZE);
}

// OK
void Encoder_RM_ZHT::SetXAB(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* out)
{
    (void)pDepth;
    memcpy(out, pAb, RM_ZHT_ABSIZE);
}

// OK
void Encoder_RM_ZHT::SetZABToNV12(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* pNV12)
{
    ZABToNV12(pDepth, pAb, pNV12);
}

// OK
void Encoder_RM_ZHT::SetXABToNV12(uint16_t const* pDepth, uint16_t const* pAb, uint8_t* pNV12)
{
    (void)pDepth;
    XABToNV12(pAb, pNV12);
}

// OK
void Encoder_RM_ZHT::BypassZ(zdepth::DepthCompressor& compressor, uint16_t const* pDepth, std::vector<uint8_t>*& pData, bool keyframe)
{
    (void)compressor;
    (void)pDepth;
    (void)keyframe;
    pData = NULL;
}

// OK
void Encoder_RM_ZHT::AppendZ(zdepth::DepthCompressor& compressor, uint16_t const* pDepth, std::vector<uint8_t>*& pData, bool keyframe)
{
    InvalidateZ(pDepth, const_cast<UINT16*>(pDepth));
    pData = new std::vector<uint8_t>(); // delete
    compressor.Compress(RM_ZHT_WIDTH, RM_ZHT_HEIGHT, pDepth, *pData, keyframe);
}

// OK
void Encoder_RM_ZHT::FreeMetadata(void* metadata, uint32_t metadata_size)
{
	(void)metadata_size;
	std::vector<uint8_t>*& z = static_cast<RM_ZHT_Metadata*>(metadata)->z;
	if (!z) { return; }
	delete z;
	z = NULL;
}

// OK
RM_ZHT_VideoTransform Encoder_RM_ZHT::GetTransform(H26xFormat const& format, ZABFormat const& zabformat)
{
	return m_vt_lut[(2 * (zabformat.profile != ZProfile::ZProfile_Same)) + (format.profile != H26xProfile::H26xProfile_None)];
}

// OK
void Encoder_RM_ZHT::SetH26xFormat(H26xFormat& format)
{
	format.width     = RM_ZHT_WIDTH;
	format.height    = RM_ZHT_HEIGHT;
	format.framerate = RM_ZHT_FPS;
}

// OK
Encoder_RM_ZHT::Encoder_RM_ZHT(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, H26xFormat const& format, ZABFormat const& zabformat, std::vector<uint64_t> const& options) :
CustomEncoder(pHookCallback, pHookParam, FreeMetadata, sizeof(RM_ZHT_Metadata), GetTransform(format, zabformat).subtype, format, RM_ZHT_WIDTH, options)
{
	m_kernel_blob = GetTransform(format, zabformat).zxx_kernel;
	m_kernel_sink = GetTransform(format, zabformat).zab_kernel;	
	m_framebytes  = GetTransform(format, zabformat).framesize;
    m_duration    = (format.divisor * HNS_BASE) / RM_ZHT_FPS;
}

// OK
void Encoder_RM_ZHT::WriteSample(UINT16 const* pDepth, UINT16 const* pAbImage, LONGLONG timestamp, RM_ZHT_Metadata* metadata)
{
    IMFMediaBuffer* pBuffer; // Release
    BYTE* pDst;

	CreateBuffer(&pBuffer, m_framebytes);

    pBuffer->Lock(&pDst, NULL, NULL);

    m_kernel_blob(m_compressor, pDepth, metadata->z, true);
    m_kernel_sink(pDepth, pAbImage, pDst);

    pBuffer->Unlock();

	WriteBuffer(pBuffer, timestamp, m_duration, metadata);

    pBuffer->Release();
}
