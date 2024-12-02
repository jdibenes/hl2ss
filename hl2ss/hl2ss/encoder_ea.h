
#pragma once

#include "custom_sink_writers.h"

#include <winrt/Windows.Media.Capture.Frames.h>

typedef void(*EA_CAST_KERNEL)(int16_t*, float   const*, int32_t);
typedef void(*EA_WIDE_KERNEL)(int16_t*, int16_t const*, int32_t);

class Encoder_EA
{
private:
    std::unique_ptr<CustomSinkWriter> m_pSinkWriter;
    uint32_t m_sample_bytes;
    uint32_t m_fill;
    EA_CAST_KERNEL m_kernel_cast;
    EA_WIDE_KERNEL m_kernel_wide;

    static void AudioF32ToS16(int16_t* out, float const* in, int32_t out_bytes);
    static void AudioS16MonoToStereo(int16_t* out, int16_t const* in, int32_t in_bytes);

    static void F32ToS16(int16_t* out, float const* in, int32_t out_bytes);
    static void S16ToS16(int16_t* out, float const* in, int32_t out_bytes);
    static void Clear(int16_t* out, float const* in, int32_t out_bytes);

    static void ExtendMonoToStereo(int16_t*, int16_t const*, int32_t);
    static void Bypass(int16_t*, int16_t const*, int32_t);

public:
    Encoder_EA(HOOK_SINK_PROC pHookCallback, void* pHookParam, AudioSubtype subtype, AACFormat& format, uint32_t channels);

    void WriteSample(winrt::Windows::Media::Capture::Frames::MediaFrameReference const& frame, int64_t timestamp);
};
