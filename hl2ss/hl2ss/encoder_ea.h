
#pragma once

#include "custom_encoder.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.h>
#include <winrt/Windows.Media.MediaProperties.h>
#include <winrt/Windows.Media.Capture.Frames.h>

typedef void(*EA_CAST_KERNEL)(int16_t*, void    const*, int32_t);
typedef void(*EA_WIDE_KERNEL)(int16_t*, int16_t const*, int32_t);

struct EA_AudioTransform
{
    EA_CAST_KERNEL cast_kernel;
    EA_WIDE_KERNEL wide_kernel;
    uint32_t sample_bytes;
    bool fill;
};

class Encoder_EA : public CustomEncoder
{
private:
    EA_CAST_KERNEL m_kernel_cast;
    EA_WIDE_KERNEL m_kernel_wide;
    uint32_t m_sample_bytes;
    uint32_t m_fill;
    
    static EA_AudioTransform const m_at_lut[4];

    static void AudioF32ToS16(int16_t* out, float const* in, int32_t samples);
    static void AudioS16MonoToStereo(int16_t* out, int16_t const* in, int32_t samples);
    static void F32ToS16(int16_t* out, void const* in, int32_t samples);
    static void S16ToS16(int16_t* out, void const* in, int32_t samples);
    static void ExtendMonoToStereo(int16_t* out, int16_t const* in, int32_t samples);
    static void Bypass(int16_t* out, int16_t const* in, int32_t samples);

    static EA_AudioTransform GetTransform(AudioSubtype subtype, uint32_t channels);

public:
    Encoder_EA(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, AudioSubtype subtype, AACFormat const& format, uint32_t channels);

    void WriteSample(winrt::Windows::Media::Capture::Frames::MediaFrameReference const& frame);

    static void SetAACFormat(AACFormat& format);
};
