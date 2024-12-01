
#pragma once

#include "custom_sink_writers.h"

class Encoder_RM_VLC
{
private:
	std::unique_ptr<CustomSinkWriter> m_pSinkWriter;
	uint32_t m_framebytes;
	uint32_t m_lumasize;
	uint32_t m_chromasize;
	LONGLONG m_duration;

public:
	Encoder_RM_VLC(HOOK_SINK_PROC pHookCallback, void* pHookParam, H26xFormat& format, std::vector<uint64_t> const& options);

	void WriteSample(BYTE const* pImage, LONGLONG timestamp, UINT8* metadata, UINT32 metadata_size);
};
