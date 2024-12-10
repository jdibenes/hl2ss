
#pragma once

#include "custom_encoder.h"

typedef void(*MC_CROP_KERNEL)(void*, void const*, int32_t);

struct MC_AudioTransform
{
	MC_CROP_KERNEL crop_kernel;
	uint32_t block_in;
	uint32_t block_out;
	AudioSubtype subtype;
};

class Encoder_MC : public CustomEncoder
{
private:
	MC_CROP_KERNEL m_kernel_crop;
	DWORD m_block_in;
	DWORD m_block_out;

	static MC_AudioTransform const m_at_lut[2];

	static void AudioF32Crop11To5(float* out, float const* in, int32_t samples);
	static void CropArray(void* out, void const* in, int32_t bytes);
	static void Bypass(void* out, void const* in, int32_t bytes);

	static MC_AudioTransform GetTransform(AACFormat const& format);

public:
	Encoder_MC(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, AACFormat const& format);

	void WriteSample(BYTE const* data, UINT32 framesAvailable, bool silent, LONGLONG timestamp);

	static void SetAACFormat(AACFormat& format, bool raw);
};
