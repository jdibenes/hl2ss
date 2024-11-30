
#include <mfapi.h>
#include "encoder_mc.h"
#include "neon.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
Encoder_MC::Encoder_MC(HOOK_SINK_PROC pHookCallback, void* pHookParam, AACFormat const& format, bool raw)
{
	AudioSubtype subtype;

	switch (raw)
	{
	case false: subtype = AudioSubtype::AudioSubtype_S16; m_block_in =  2 * sizeof(uint16_t); m_block_out = 2 * sizeof(uint16_t); break;
	default:    subtype = AudioSubtype::AudioSubtype_F32; m_block_in = 11 * sizeof(float);    m_block_out = 5 * sizeof(float);    break;
	}

	m_pSinkWriter = CustomSinkWriter::CreateForAudio(pHookCallback, pHookParam, subtype, format);
	m_raw = raw;
}

// OK
void Encoder_MC::WriteSample(BYTE* data, UINT32 framesAvailable, bool silent, LONGLONG timestamp)
{
	int const frame_duration_num = 625; // 10,000,000
	int const frame_duration_den = 3;   //     48,000

	DWORD bytes_in  = framesAvailable * m_block_in;
	DWORD bytes_out = framesAvailable * m_block_out;
	IMFSample* pSample; // Release
	IMFMediaBuffer* pBuffer; // Release
	BYTE* pDst;

    MFCreateMemoryBuffer(bytes_out, &pBuffer);

    pBuffer->Lock(&pDst, NULL, NULL);
	if (silent) { memset(pDst, 0, bytes_out); } else if (m_raw) { Neon_F32CropAudio11to5((float*)data, bytes_in / sizeof(float), (float*)pDst); } else { memcpy(pDst, data, bytes_in); }
	pBuffer->Unlock();
	pBuffer->SetCurrentLength(bytes_out);

	MFCreateSample(&pSample);

	pSample->AddBuffer(pBuffer);
	pSample->SetSampleDuration((framesAvailable * frame_duration_num) / frame_duration_den);
	pSample->SetSampleTime(timestamp);

	m_pSinkWriter->WriteSample(pSample);

	pBuffer->Release();
	pSample->Release();
}
