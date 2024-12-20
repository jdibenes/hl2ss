
#include "encoder_ez.h"

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Storage::Streams;

// OK
Encoder_EZ::Encoder_EZ(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, H26xFormat const& format, ZABFormat const& zabformat)
{
    m_hook   = pHookCallback;
    m_param  = pHookParam;
    m_width  = format.width;
    m_height = format.height;
    m_raw    = zabformat.profile == ZProfile::ZProfile_Same;
}

// OK
bool Encoder_EZ::WriteSample(MediaFrameReference const& frame, EZ_Metadata* metadata)
{
    std::vector<uint8_t> encoded;
    void* data;
    DWORD size;

    auto buffer = frame.BufferMediaFrame();
    if (!buffer) { return false; }

    auto b = buffer.Buffer();
    if (b.Length() < (m_width * m_height * sizeof(UINT16))) { return false; }

    if (!m_raw)
    {
    zdepth::DepthResult result = m_compressor.Compress(m_width, m_height, reinterpret_cast<uint16_t*>(b.data()), encoded, true);
    if (result != zdepth::DepthResult::Success) { return false; }
    data = encoded.data();
    size = static_cast<DWORD>(encoded.size());
    }
    else
    {
    data = b.data();
    size = b.Length();
    }
    
    m_hook(data, size, TRUE, frame.SystemRelativeTime().Value().count(), metadata, sizeof(EZ_Metadata), m_param);

    return true;
}
