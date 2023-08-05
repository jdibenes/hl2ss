
#pragma once

#include <mfidl.h>

int const RAW_PROFILE = 0xFF;

enum H26xProfile
{
    H264Profile_Base,
    H264Profile_Main,
    H264Profile_High,
    H265Profile_Main,
    H26xProfile_None = RAW_PROFILE
};

enum AACProfile
{
    AACProfile_12000,
    AACProfile_16000,
    AACProfile_20000,
    AACProfile_24000,
    AACProfile_None = RAW_PROFILE
};

HRESULT CreateTypePCMF32(IMFMediaType** ppType, uint32_t channels, uint32_t samplerate);
HRESULT CreateTypePCMS16(IMFMediaType** ppType, uint32_t channels, uint32_t samplerate);
HRESULT CreateTypeAAC(IMFMediaType** ppType, uint32_t channels, uint32_t samplerate, AACProfile profile);
HRESULT CreateTypeL8(IMFMediaType** ppType, uint32_t width, uint32_t height, uint32_t stride, uint32_t fps);
HRESULT CreateTypeNV12(IMFMediaType** ppType, uint32_t width, uint32_t height, uint32_t stride, uint32_t fps);
HRESULT CreateTypeARGB(IMFMediaType** ppType, uint32_t width, uint32_t height, uint32_t stride, uint32_t fps);
HRESULT CreateTypeH26x(IMFMediaType** ppType, uint32_t width, uint32_t height, uint32_t fps, H26xProfile profile, uint32_t bitrate);
