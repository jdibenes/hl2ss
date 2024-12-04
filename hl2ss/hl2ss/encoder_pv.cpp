
#include "encoder_pv.h"
#include "custom_media_buffers.h"
#include "timestamps.h"

#include <winrt/Windows.Media.Capture.Frames.h>

using namespace winrt::Windows::Media::Capture::Frames;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
Encoder_PV::Encoder_PV(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, VideoSubtype subtype, H26xFormat const& format, uint32_t stride, std::vector<uint64_t> const& options) :
CustomEncoder(pHookCallback, pHookParam, NULL, sizeof(PV_Metadata), subtype, format, stride, options)
{
    m_duration = (format.divisor * HNS_BASE) / format.framerate;
}

// OK
void Encoder_PV::WriteSample(MediaFrameReference const& frame, LONGLONG timestamp, PV_Metadata* metadata)
{
    SoftwareBitmapBuffer* pBuffer; // Release
    SoftwareBitmapBuffer::CreateInstance(&pBuffer, frame);
    WriteBuffer(pBuffer, timestamp, m_duration, reinterpret_cast<UINT8*>(metadata));
    pBuffer->Release();
}
