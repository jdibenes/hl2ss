
#pragma once

#include "custom_sink_writers.h"

#include <winrt/Windows.Media.Capture.Frames.h>

class Encoder_PV
{
private:
    std::unique_ptr<CustomSinkWriter> m_pSinkWriter;
    LONGLONG m_duration;

public:
    Encoder_PV(HOOK_SINK_PROC pHookCallback, void* pHookParam, VideoSubtype subtype, H26xFormat const& format, uint32_t stride, std::vector<uint64_t> const& options);

    void WriteSample(winrt::Windows::Media::Capture::Frames::MediaFrameReference const& frame, LONGLONG timestamp, UINT8* metadata, UINT32 metadata_size);
};
