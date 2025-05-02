
#include "custom_media_buffers.h"
#include "encoder_ez.h"

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Storage::Streams;

// OK
Encoder_EZ::Encoder_EZ(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, H26xFormat const& format, ZABFormat const& zabformat) :
CustomEncoder(pHookCallback, pHookParam, NULL, sizeof(EZ_Metadata))
{
    m_width  = format.width;
    m_height = format.height;
    m_raw    = zabformat.profile == ZProfile::ZProfile_Same;
}

// OK
bool Encoder_EZ::WriteSample(MediaFrameReference const& frame, EZ_Metadata* metadata)
{
    VectorBuffer* pBuffer; // Release

    auto buffer = frame.BufferMediaFrame();
    if (!buffer) { return false; }

    auto b = buffer.Buffer();
    if (b.Length() < (m_width * m_height * sizeof(UINT16))) { return false; }

    VectorBuffer::CreateInstance(&pBuffer);
    std::vector<uint8_t>& v = pBuffer->Get();
    bool ok;

    if (!m_raw)
    {
    zdepth::DepthResult result = m_compressor.Compress(m_width, m_height, reinterpret_cast<uint16_t*>(b.data()), v, true);
    ok = result == zdepth::DepthResult::Success;
    }
    else
    {
    v.resize(b.Length());
    memcpy(v.data(), b.data(), b.Length());
    ok = true;
    }

    if (ok) { WriteBuffer(pBuffer, frame.SystemRelativeTime().Value().count(), 0, metadata); }
    pBuffer->Release();

    return ok;
}
