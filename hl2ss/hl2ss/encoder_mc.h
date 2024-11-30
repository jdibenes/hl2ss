
#pragma once

#include "custom_sink_writers.h"

class Encoder_MC
{
private:
	std::unique_ptr<CustomSinkWriter> m_pSinkWriter;
	bool m_raw;
	DWORD m_block_in;
	DWORD m_block_out;

public:
	Encoder_MC(HOOK_SINK_PROC pHookCallback, void* pHookParam, AACFormat const& format, bool raw);

	void WriteSample(BYTE* data, UINT32 framesAvailable, bool silent, LONGLONG timestamp);
};
