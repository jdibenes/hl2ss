
#pragma once

#include "custom_sink_writers.h"

#include <winrt/Windows.Media.Capture.Frames.h>

class Encoder_EA
{
private:
    std::unique_ptr<CustomSinkWriter> m_pSinkWriter;

public:
    Encoder_EA(HOOK_SINK_PROC pHookCallback, void* pHookParam, AACFormat const& format);

    void WriteSample(winrt::Windows::Media::Capture::Frames::MediaFrameReference const& frame, int64_t timestamp);
};
