
#pragma once

#include "custom_sink_writers.h"

typedef void(*MC_CROP_KERNEL)(void*, void const*, int32_t);

class Encoder_MC
{
private:
	std::unique_ptr<CustomSinkWriter> m_pSinkWriter;
	MC_CROP_KERNEL m_kernel_crop;
	DWORD m_block_in;
	DWORD m_block_out;

	static void AudioF32Crop11To5(float* out, float const* in, int32_t bytes);

	static void CropArray(void* out, void const* in, int32_t bytes);
	static void Bypass(void* out, void const* in, int32_t bytes);

public:
	Encoder_MC(HOOK_SINK_PROC pHookCallback, void* pHookParam, AACFormat& format, bool raw);

	void WriteSample(BYTE const* data, UINT32 framesAvailable, bool silent, LONGLONG timestamp);
};
