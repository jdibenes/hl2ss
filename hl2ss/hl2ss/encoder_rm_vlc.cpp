
#include <mfapi.h>
#include "encoder_rm_vlc.h"
#include "research_mode.h"
#include "timestamps.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
Encoder_RM_VLC::Encoder_RM_VLC(HOOK_SINK_PROC pHookCallback, void* pHookParam, H26xFormat& format, std::vector<uint64_t> const& options)
{
	VideoSubtype subtype;

	format.width     = RM_VLC_WIDTH;
	format.height    = RM_VLC_HEIGHT;
	format.framerate = RM_VLC_FPS;

	switch (format.profile)
	{
	case H26xProfile::H26xProfile_None: m_chromasize = 0;                 subtype = VideoSubtype::VideoSubtype_L8;   break;
	default:                            m_chromasize = RM_VLC_PIXELS / 2; subtype = VideoSubtype::VideoSubtype_NV12; break;
	}

	m_lumasize   = RM_VLC_PIXELS;
	m_framebytes = m_lumasize + m_chromasize;
	m_duration   = (format.divisor * HNS_BASE) / RM_VLC_FPS;

	m_pSinkWriter = CustomSinkWriter::CreateForVideo(pHookCallback, pHookParam, subtype, format, RM_VLC_WIDTH, options);
}

// OK
void Encoder_RM_VLC::WriteSample(BYTE const* pImage, LONGLONG timestamp, UINT8* metadata, UINT32 metadata_size)
{
	IMFMediaBuffer* pBuffer; // Release
	IMFSample* pSample; // Release
	BYTE* pDst;

	MFCreateMemoryBuffer(m_framebytes, &pBuffer);

	pBuffer->Lock(&pDst, NULL, NULL);
	memcpy(pDst,              pImage,           m_lumasize);
	memset(pDst + m_lumasize, NV12_ZERO_CHROMA, m_chromasize);
	pBuffer->Unlock();
	pBuffer->SetCurrentLength(m_framebytes);

	MFCreateSample(&pSample);

	pSample->AddBuffer(pBuffer);
	pSample->SetSampleDuration(m_duration);
	pSample->SetSampleTime(timestamp);
	pSample->SetBlob(MF_USER_DATA_PAYLOAD, metadata, metadata_size);

	m_pSinkWriter->WriteSample(pSample);

	pSample->Release();
	pBuffer->Release();
}
