
#include "encoder_pv.h"
#include "custom_media_buffers.h"

using namespace winrt::Windows::Media::Capture::Frames;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
Encoder_PV::Encoder_PV(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, VideoSubtype subtype, H26xFormat const& format, uint32_t stride, std::vector<uint64_t> const& options) :
CustomEncoder(pHookCallback, pHookParam, NULL, sizeof(PV_Metadata), subtype, format, stride, options)
{
}

// OK
void Encoder_PV::WriteSample(MediaFrameReference const& frame, PV_Metadata* metadata)
{
    SoftwareBitmapBuffer* pBuffer; // Release
    SoftwareBitmapBuffer::CreateInstance(&pBuffer, frame);
    WriteBuffer(pBuffer, frame.SystemRelativeTime().Value().count(), frame.Duration().count(), metadata);
    pBuffer->Release();
}
