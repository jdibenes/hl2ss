
#include "encoder_mc.h"

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

MC_AudioTransform const Encoder_MC::m_at_lut[2] =
{
	{Bypass,     2 * sizeof(int16_t), 2 * sizeof(int16_t), AudioSubtype::AudioSubtype_S16},
	{CropArray, 11 * sizeof(float),   5 * sizeof(float),   AudioSubtype::AudioSubtype_F32}
};

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Encoder_MC::AudioF32Crop11To5(float* out, float const* in, int32_t samples)
{
	for (int i = 0; i < (samples / 4); ++i)
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
void Encoder_MC::CropArray(void* out, void const* in, int32_t bytes)
{
	AudioF32Crop11To5(static_cast<float*>(out), static_cast<float const*>(in), bytes / sizeof(float));
}

// OK
void Encoder_MC::Bypass(void* out, void const* in, int32_t bytes)
{
	memcpy(out, in, bytes);
}

// OK
MC_AudioTransform Encoder_MC::GetTransform(AACFormat const& format)
{
	return m_at_lut[format.channels == 5];
}

// OK
void Encoder_MC::SetAACFormat(AACFormat& format, bool raw)
{
	format.samplerate = 48000;
	format.channels   = raw ? 5 : 2;
}

// OK
Encoder_MC::Encoder_MC(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, AACFormat const& format) :
CustomEncoder(pHookCallback, pHookParam, NULL, 0, GetTransform(format).subtype, format)
{
	m_kernel_crop = GetTransform(format).crop_kernel;
	m_block_in    = GetTransform(format).block_in;
	m_block_out   = GetTransform(format).block_out;
}

// OK
void Encoder_MC::WriteSample(BYTE const* data, UINT32 framesAvailable, bool silent, LONGLONG timestamp)
{
	int const frame_duration_num = 625; // 10,000,000
	int const frame_duration_den = 3;   //     48,000

	DWORD bytes_in  = framesAvailable * m_block_in;
	DWORD bytes_out = framesAvailable * m_block_out;
	IMFMediaBuffer* pBuffer; // Release
	BYTE* pDst;

	CreateBuffer(&pBuffer, bytes_out);

    pBuffer->Lock(&pDst, NULL, NULL);

	if (silent) { memset(pDst, 0, bytes_out); }	else { m_kernel_crop(pDst, data, bytes_in); }

	pBuffer->Unlock();

	WriteBuffer(pBuffer, timestamp, (framesAvailable * frame_duration_num) / frame_duration_den, NULL);

	pBuffer->Release();
}
