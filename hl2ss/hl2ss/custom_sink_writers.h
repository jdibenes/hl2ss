
#pragma once

#include <memory>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <vector>
#include "custom_media_types.h"
#include "custom_media_sink.h"

#include <winrt/Windows.Graphics.Imaging.h>

struct H26xFormat
{
    uint16_t    width;
    uint16_t    height;
    uint8_t     framerate;
    uint8_t     divisor;
    H26xProfile profile;
    int8_t      level;
    uint32_t    bitrate;
};

struct AACFormat
{
    uint16_t   samplerate;
    uint8_t    channels;    
    AACProfile profile;
    AACLevel   level;
    uint8_t    _reserved[3];
};

struct ZABFormat
{
    winrt::Windows::Graphics::Imaging::PngFilterMode filter;
    ZProfile profile;
    uint8_t _reserved[3];
};

class CustomSinkWriter
{
private:
    CustomMediaSink* m_pSink; // Release
    IMFSinkWriter* m_pSinkWriter; // Release
    DWORD m_dwStreamIndex;

public:
    CustomSinkWriter(HOOK_SINK_PROC hookproc, void* hookparam, IMFMediaType* pInputType, IMFMediaType* pOutputType, IMFAttributes* pEncoderAttr);
    virtual ~CustomSinkWriter();

    void WriteSample(IMFSample* pSample);

    static std::unique_ptr<CustomSinkWriter> CreateForAudio(HOOK_SINK_PROC hookproc, void* hookparam, AudioSubtype input_subtype, AACFormat  const& format);
    static std::unique_ptr<CustomSinkWriter> CreateForVideo(HOOK_SINK_PROC hookproc, void* hookparam, VideoSubtype input_subtype, H26xFormat const& format, uint32_t stride, std::vector<uint64_t> const& encoder_options);
};
