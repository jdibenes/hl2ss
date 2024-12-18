
#include "encoder_ez.h"

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Storage::Streams;

// OK
Encoder_EZ::Encoder_EZ(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, H26xFormat const& format)
{
    m_hook   = pHookCallback;
    m_param  = pHookParam;
    m_width  = format.width;
    m_height = format.height;
}

// OK
bool Encoder_EZ::WriteSample(MediaFrameReference const& frame, EZ_Metadata* metadata)
{
    std::vector<uint8_t> encoded;

    {
    auto buffer = frame.BufferMediaFrame();
    if (!buffer) { return false; }

    auto b = buffer.Buffer();
    if (b.Length() < (m_width * m_height * sizeof(UINT16))) { return false; }

    zdepth::DepthResult result = m_compressor.Compress(m_width, m_height, reinterpret_cast<uint16_t*>(b.data()), encoded, true);
    if (result != zdepth::DepthResult::Success) { return false; }
    }
    
    m_hook(encoded.data(), static_cast<DWORD>(encoded.size()), TRUE, frame.SystemRelativeTime().Value().count(), metadata, sizeof(EZ_Metadata), m_param);

    return true;
}
