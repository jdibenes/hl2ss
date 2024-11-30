
#include <mfapi.h>
#include "encoder_ea.h"
#include "neon.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.h>
#include <winrt/Windows.Media.MediaProperties.h>
#include <winrt/Windows.Media.Capture.Frames.h>

using namespace winrt::Windows::Media;
using namespace winrt::Windows::Media::Capture::Frames;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
Encoder_EA::Encoder_EA(HOOK_SINK_PROC pHookCallback, void* pHookParam, AACFormat const& format)
{
    m_pSinkWriter = CustomSinkWriter::CreateForAudio(pHookCallback, pHookParam, AudioSubtype::AudioSubtype_S16, format);
}

// OK
void Encoder_EA::WriteSample(MediaFrameReference const& frame, int64_t timestamp)
{
    IMFSample* pSample; // Release
    IMFMediaBuffer* pBuffer; // Release
    BYTE* pDst;

    AudioMediaFrame amf       = frame.AudioMediaFrame();
    AudioFrame      audio     = amf.GetAudioFrame();
    AudioBuffer     buffer    = audio.LockBuffer(AudioBufferAccessMode::Read);
    auto const&     reference = buffer.CreateReference();
    auto const&     aep       = amf.AudioEncodingProperties();

    uint32_t bytes_per_sample = aep.BitsPerSample() / 8;
    uint32_t channels         = aep.ChannelCount();

    uint32_t input_bytes   = reference.Capacity();
    uint32_t input_samples = input_bytes / bytes_per_sample;
    uint32_t output_bytes  = input_samples * sizeof(int16_t);
    uint32_t fill_bytes    = (channels == 1) ? output_bytes : 0;
    uint32_t buffer_bytes  = output_bytes + fill_bytes;

    MFCreateMemoryBuffer(buffer_bytes, &pBuffer);

    pBuffer->Lock(&pDst, NULL, NULL);

    float*   src_addr  = (float*)reference.data();
    int16_t* base_addr = (int16_t*)pDst;
    int16_t* high_addr = (int16_t*)(pDst + fill_bytes);

    switch (bytes_per_sample)
    {
    case 4:  Neon_F32ToS16(src_addr, input_samples, high_addr); break;
    case 2:  memcpy(high_addr, src_addr, output_bytes);         break;
    default: memset(high_addr, 0, output_bytes);
    }

    if (channels == 1) { Neon_S16MonoToStereo(high_addr, input_samples, base_addr); }

    pBuffer->Unlock();

    pBuffer->SetCurrentLength(buffer_bytes);

    reference.Close();
    buffer.Close();
    audio.Close();

    MFCreateSample(&pSample);

    pSample->AddBuffer(pBuffer);
    pSample->SetSampleDuration(frame.Duration().count());
    pSample->SetSampleTime(timestamp);

    m_pSinkWriter->WriteSample(pSample);

    pBuffer->Release();
    pSample->Release();
}
