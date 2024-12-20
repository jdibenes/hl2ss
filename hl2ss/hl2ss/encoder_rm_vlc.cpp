
#include "encoder_rm_vlc.h"
#include "research_mode.h"
#include "timestamp.h"

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

RM_VLC_VideoTransform const Encoder_RM_VLC::m_vt_lut[2] =
{
	{0,                 VideoSubtype::VideoSubtype_L8},
	{RM_VLC_PIXELS / 2, VideoSubtype::VideoSubtype_NV12}
};

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
RM_VLC_VideoTransform Encoder_RM_VLC::GetTransform(H26xFormat const& format)
{
	return m_vt_lut[format.profile != H26xProfile::H26xProfile_None];
}

// OK
void Encoder_RM_VLC::SetH26xFormat(H26xFormat& format)
{
	format.width     = RM_VLC_WIDTH;
	format.height    = RM_VLC_HEIGHT;
	format.framerate = RM_VLC_FPS;
}

// OK
Encoder_RM_VLC::Encoder_RM_VLC(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, H26xFormat const& format, std::vector<uint64_t> const& options) :
CustomEncoder(pHookCallback, pHookParam, NULL, sizeof(RM_VLC_Metadata), GetTransform(format).subtype, format, RM_VLC_WIDTH, options)
{
	m_lumasize   = RM_VLC_PIXELS;
	m_chromasize = GetTransform(format).chromasize;	
	m_framebytes = m_lumasize + m_chromasize;
	m_duration   = (format.divisor * HNS_BASE) / RM_VLC_FPS;
}

// OK
void Encoder_RM_VLC::WriteSample(BYTE const* pImage, LONGLONG timestamp, RM_VLC_Metadata* metadata)
{
	IMFMediaBuffer* pBuffer; // Release
	BYTE* pDst;

	CreateBuffer(&pBuffer, m_framebytes);

	pBuffer->Lock(&pDst, NULL, NULL);

	memcpy(pDst,              pImage,           m_lumasize);
	memset(pDst + m_lumasize, NV12_ZERO_CHROMA, m_chromasize);

	pBuffer->Unlock();

	WriteBuffer(pBuffer, timestamp, m_duration, metadata);

	pBuffer->Release();
}
