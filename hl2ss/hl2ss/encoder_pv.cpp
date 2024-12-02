
#include <mfapi.h>
#include "encoder_pv.h"
#include "custom_media_buffers.h"
#include "timestamps.h"

#include <winrt/Windows.Media.Capture.Frames.h>

using namespace winrt::Windows::Media::Capture::Frames;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
Encoder_PV::Encoder_PV(HOOK_SINK_PROC pHookCallback, void* pHookParam, VideoSubtype subtype, H26xFormat const& format, uint32_t stride, std::vector<uint64_t> const& options)
{
    m_duration = (format.divisor * HNS_BASE) / format.framerate;
    m_pSinkWriter = CustomSinkWriter::CreateForVideo(pHookCallback, pHookParam, subtype, format, stride, options);
}

// OK
void Encoder_PV::WriteSample(MediaFrameReference const& frame, LONGLONG timestamp, UINT8* metadata, UINT32 metadata_size)
{
    SoftwareBitmapBuffer* pBuffer; // Release
    IMFSample* pSample; // Release

    SoftwareBitmapBuffer::CreateInstance(&pBuffer, frame);

    MFCreateSample(&pSample);

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(m_duration);
    pSample->SetSampleTime(timestamp);
    pSample->SetBlob(MF_USER_DATA_PAYLOAD, metadata, metadata_size);

    m_pSinkWriter->WriteSample(pSample);

    pSample->Release();
    pBuffer->Release();
}
