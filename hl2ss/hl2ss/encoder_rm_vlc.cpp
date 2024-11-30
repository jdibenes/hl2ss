
#include <mfapi.h>
#include "encoder_rm_vlc.h"
#include "research_mode.h"
#include "timestamps.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
Encoder_RM_VLC::Encoder_RM_VLC(HOOK_SINK_PROC pHookCallback, void* pHookParam, H26xFormat const& format, std::vector<uint64_t> const& options)
{
	bool         encoded = format.profile != H26xProfile::H26xProfile_None;
	VideoSubtype subtype = encoded ? VideoSubtype::VideoSubtype_NV12 : VideoSubtype::VideoSubtype_L8;

	m_lumasize   = format.width * format.height;
	m_chromasize = encoded ? (m_lumasize / 2) : 0;
	m_framebytes = m_lumasize + m_chromasize;
	m_duration   = (format.divisor * HNS_BASE) / format.framerate;

	m_pSinkWriter = CustomSinkWriter::CreateForVideo(pHookCallback, pHookParam, subtype, format, format.width, options);
}

// OK
void Encoder_RM_VLC::WriteSample(BYTE const* pImage, LONGLONG timestamp, UINT8* metadata, UINT32 metadata_size) const
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
