
#pragma once

#include <mfidl.h>

enum H26xProfile
{
    H264Profile_Base,
    H264Profile_Main,
    H264Profile_High,
    H265Profile_Main
};

enum AACBitrate
{
    AACBitrate_12000,
    AACBitrate_16000,
    AACBitrate_20000,
    AACBitrate_24000
};

HRESULT CreateTypePCMF32(IMFMediaType** ppType, uint32_t channels, uint32_t samplerate);
HRESULT CreateTypePCMS16(IMFMediaType** ppType, uint32_t channels, uint32_t samplerate);
HRESULT CreateTypeAAC(IMFMediaType** ppType, uint32_t channels, uint32_t samplerate, AACBitrate bitrate);
HRESULT CreateTypeNV12(IMFMediaType** ppType, uint32_t width, uint32_t height, uint32_t stride, uint32_t fps);
HRESULT CreateTypeH26x(IMFMediaType** ppType, uint32_t width, uint32_t height, uint32_t fps, H26xProfile profile, uint32_t bitrate);
