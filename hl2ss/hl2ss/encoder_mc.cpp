
#include <mfapi.h>
#include "encoder_mc.h"

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Encoder_MC::AudioF32Crop11to5(float const* in, int32_t bytes, float *out)
{
	for (int i = 0; i < (bytes / (sizeof(float) * 4)); ++i)
	{
	float32x4_t f = vld1q_f32(in);
	switch (i % 11)
	{
	case 0:
		out[0] = f.n128_f32[0];
		out[1] = f.n128_f32[1];
		out[2] = f.n128_f32[2];
		out[3] = f.n128_f32[3];
		out += 4;
		break;
	case 1:
		out[0] = f.n128_f32[0];
		out += 1;
		break;
	case 2:
		out[0] = f.n128_f32[3];
		out += 1;
		break;
	case 3:
		out[0] = f.n128_f32[0];
		out[1] = f.n128_f32[1];
		out[2] = f.n128_f32[2];
		out[3] = f.n128_f32[3];
		out += 4;
		break;
	case 4:
		break;
	case 5:
		out[0] = f.n128_f32[2];
		out[1] = f.n128_f32[3];
		out += 2;
		break;
	case 6:
		out[0] = f.n128_f32[0];
		out[1] = f.n128_f32[1];
		out[2] = f.n128_f32[2];
		out += 3;
		break;
	case 7:
		break;
	case 8:
		out[0] = f.n128_f32[1];
		out[1] = f.n128_f32[2];
		out[2] = f.n128_f32[3];
		out += 3;
		break;
	case 9:
		out[0] = f.n128_f32[0];
		out[1] = f.n128_f32[1];
		out += 2;
		break;
	case 10:
		break;
	}
	in += 4;
	}
}

// OK
Encoder_MC::Encoder_MC(HOOK_SINK_PROC pHookCallback, void* pHookParam, AACFormat& format, bool raw)
{
	AudioSubtype subtype;
	uint8_t channels;

	switch (raw)
	{
	case false: channels = 2; subtype = AudioSubtype::AudioSubtype_S16; m_block_in =  2 * sizeof(uint16_t); m_block_out = 2 * sizeof(uint16_t); break;
	default:    channels = 5; subtype = AudioSubtype::AudioSubtype_F32; m_block_in = 11 * sizeof(float);    m_block_out = 5 * sizeof(float);    break;
	}

	format.samplerate = 48000;
	format.channels   = channels;

	m_raw = raw;
	m_pSinkWriter = CustomSinkWriter::CreateForAudio(pHookCallback, pHookParam, subtype, format);
}

// OK
void Encoder_MC::WriteSample(BYTE const* data, UINT32 framesAvailable, bool silent, LONGLONG timestamp)
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
	if (silent) { memset(pDst, 0, bytes_out); } else if (m_raw) { AudioF32Crop11to5((float*)data, bytes_in, (float*)pDst); } else { memcpy(pDst, data, bytes_in); }
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
