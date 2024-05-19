
#pragma once

#include <mfidl.h>

uint8_t const RAW_PROFILE = 0xFF;
int32_t const H26xLevel_Default = -1;

enum AACProfile : uint8_t
{
    AACProfile_12000,
    AACProfile_16000,
    AACProfile_20000,
    AACProfile_24000,
    AACProfile_None = RAW_PROFILE
};

enum AACLevel : uint8_t
{
    AACLevel_L2      = 0x29,
    AACLevel_L4      = 0x2A,
    AACLevel_L5      = 0x2B,
    AACLevel_HEv1L2  = 0x2C,
    AACLevel_HEv1L4  = 0x2E,
    AACLevel_HEv1L5  = 0x2F,
    AACLevel_HEv2L2  = 0x30,
    AACLevel_HEv2L3  = 0x31,
    AACLevel_HEv2L4  = 0x32,
    AACLevel_HEv2L5  = 0x33,
    AACLevel_Default = AACLevel_L2
};

enum AudioSubtype : uint8_t
{
    AudioSubtype_F32,
    AudioSubtype_S16
};

enum H26xProfile : uint8_t
{
    H264Profile_Base,
    H264Profile_Main,
    H264Profile_High,
    H265Profile_Main,
    H26xProfile_None = RAW_PROFILE
};

enum VideoSubtype : uint8_t
{
    VideoSubtype_L8,
    VideoSubtype_L16,
    VideoSubtype_NV12,
    VideoSubtype_ARGB,
    VideoSubtype_YUY2,
    VideoSubtype_IYUV,
    VideoSubtype_YV12
};

HRESULT CreateTypeAudio(IMFMediaType** ppType, uint32_t channels, uint32_t samplerate, AudioSubtype subtype, AACProfile profile, AACLevel level);
HRESULT CreateTypeVideo(IMFMediaType** ppType, uint32_t width, uint32_t height, uint32_t stride, uint32_t fps_num, uint32_t fps_den, VideoSubtype subtype, H26xProfile profile, int32_t level, uint32_t bitrate);
