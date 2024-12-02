
#include <mfapi.h>
#include "encoder_rm_zht.h"
#include "research_mode.h"
#include "timestamps.h"

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
    InvalidateZ(pDepth, (uint16_t*)out);
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
Encoder_RM_ZHT::Encoder_RM_ZHT(HOOK_SINK_PROC pHookCallback, void* pHookParam, H26xFormat& format, ZABFormat& zabformat, std::vector<uint64_t> const& options)
{
    VideoSubtype subtype;

	format.width     = RM_ZHT_WIDTH;
	format.height    = RM_ZHT_HEIGHT;
	format.framerate = RM_ZHT_FPS;
	
    switch (2 * (zabformat.profile != ZProfile::ZProfile_Same) + (format.profile != H26xProfile::H26xProfile_None))
    {
    case 0:  m_kernel_sink = SetZAB;       m_kernel_blob = BypassZ; m_framebytes =  RM_ZHT_PIXELS * 4;      subtype = VideoSubtype::VideoSubtype_ARGB; break;
    case 1:  m_kernel_sink = SetZABToNV12; m_kernel_blob = BypassZ; m_framebytes = (RM_ZHT_PIXELS * 3) / 2; subtype = VideoSubtype::VideoSubtype_NV12; break;
    case 2:  m_kernel_sink = SetXAB;       m_kernel_blob = AppendZ; m_framebytes =  RM_ZHT_PIXELS * 2;      subtype = VideoSubtype::VideoSubtype_L16;  break;
    default: m_kernel_sink = SetXABToNV12; m_kernel_blob = AppendZ; m_framebytes = (RM_ZHT_PIXELS * 3) / 2; subtype = VideoSubtype::VideoSubtype_NV12; break;
    }

    m_duration = (format.divisor * HNS_BASE) / RM_ZHT_FPS;

    m_pSinkWriter = CustomSinkWriter::CreateForVideo(pHookCallback, pHookParam, subtype, format, RM_ZHT_WIDTH, options);
}

// OK
void Encoder_RM_ZHT::WriteSample(UINT16 const* pDepth, UINT16 const* pAbImage, LONGLONG timestamp, RM_ZHT_SLOT* metadata, UINT32 metadata_size)
{
    IMFMediaBuffer* pBuffer; // Release
    IMFSample* pSample; // Release
    BYTE* pDst;

    MFCreateMemoryBuffer(m_framebytes, &pBuffer);

    pBuffer->Lock(&pDst, NULL, NULL);
    m_kernel_blob(m_compressor, pDepth, metadata->z, true);
    m_kernel_sink(pDepth, pAbImage, pDst);
    pBuffer->Unlock();
    pBuffer->SetCurrentLength(m_framebytes);

    MFCreateSample(&pSample);

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(m_duration);
    pSample->SetSampleTime(timestamp);
    pSample->SetBlob(MF_USER_DATA_PAYLOAD, (UINT8*)metadata, metadata_size);

    m_pSinkWriter->WriteSample(pSample);

    pSample->Release();
    pBuffer->Release();
}
