
#include <mfapi.h>
#include "encoder_ea.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Media.h>
#include <winrt/Windows.Media.MediaProperties.h>
#include <winrt/Windows.Media.Capture.Frames.h>

using namespace winrt::Windows::Media;
using namespace winrt::Windows::Media::MediaProperties;
using namespace winrt::Windows::Media::Capture::Frames;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Encoder_EA::AudioF32ToS16(int16_t* out, float const* in, int32_t out_bytes)
{
	float32x4_t s = vdupq_n_f32(32767.0f);

	for (int i = 0; i < (out_bytes / (sizeof(int16_t) * 16)); ++i)
	{
	float32x4x4_t f = vld1q_f32_x4(in);
	uint16x4x4_t d;

	d.val[0] = vmovn_s32(vcvtq_s32_f32(vmulq_f32(f.val[0], s)));
	d.val[1] = vmovn_s32(vcvtq_s32_f32(vmulq_f32(f.val[1], s)));
	d.val[2] = vmovn_s32(vcvtq_s32_f32(vmulq_f32(f.val[2], s)));
	d.val[3] = vmovn_s32(vcvtq_s32_f32(vmulq_f32(f.val[3], s)));

	vst1_u16_x4(out, d);

	in  += 16;
	out += 16;
	}
}

// OK
void Encoder_EA::AudioS16MonoToStereo(int16_t* out, int16_t const* in, int32_t in_bytes)
{
	for (int i = 0; i < (in_bytes / (sizeof(int16_t) * 32)); ++i)
	{
	int16x8x4_t f = vld1q_s16_x4(in);

	for (int j = 0; j < 4; ++j)
	{
	int16x8x2_t d;

	d.val[0] = f.val[j];
	d.val[1] = f.val[j];

	vst2q_s16(out, d);
    
	out += 16;
	}

	in  += 32;
	}
}

// OK
void Encoder_EA::F32ToS16(int16_t* out, float const* in, int32_t out_bytes)
{
    AudioF32ToS16(out, in, out_bytes);
}

// OK
void Encoder_EA::S16ToS16(int16_t* out, float const* in, int32_t out_bytes)
{
    memcpy(out, in, out_bytes);
}

// OK
void Encoder_EA::Clear(int16_t* out, float const* in, int32_t out_bytes)
{
    (void)in;
    memset(out, 0, out_bytes);
}

// OK
void Encoder_EA::ExtendMonoToStereo(int16_t* out, int16_t const* in, int32_t in_bytes)
{
    AudioS16MonoToStereo(out, in, in_bytes);
}

// OK
void Encoder_EA::Bypass(int16_t* out, int16_t const* in, int32_t in_bytes)
{
    (void)out;
    (void)in;
    (void)in_bytes;
}

// OK
Encoder_EA::Encoder_EA(HOOK_SINK_PROC pHookCallback, void* pHookParam, AudioSubtype subtype, AACFormat& format, uint32_t channels)
{
    format.samplerate = 48000;
    format.channels   = 2;

    switch (subtype)
    {
    case AudioSubtype::AudioSubtype_S16: m_kernel_cast = S16ToS16; m_sample_bytes = sizeof(int16_t); break;
    case AudioSubtype::AudioSubtype_F32: m_kernel_cast = F32ToS16; m_sample_bytes = sizeof(float);   break;
    default:                             m_kernel_cast = Clear;    m_sample_bytes = sizeof(int16_t); break;
    }

    switch (channels)
    {
    case 1:  m_kernel_wide = ExtendMonoToStereo; m_fill = 1; break;
    default: m_kernel_wide = Bypass;             m_fill = 0; break;
    }

    m_pSinkWriter = CustomSinkWriter::CreateForAudio(pHookCallback, pHookParam, AudioSubtype::AudioSubtype_S16, format);
}

// OK
void Encoder_EA::WriteSample(MediaFrameReference const& frame, int64_t timestamp)
{
    IMFSample* pSample; // Release
    IMFMediaBuffer* pBuffer; // Release
    BYTE* pDst;

    auto amf       = frame.AudioMediaFrame();
    auto audio     = amf.GetAudioFrame();
    auto buffer    = audio.LockBuffer(AudioBufferAccessMode::Read);
    auto reference = buffer.CreateReference();

    uint32_t output_bytes = (reference.Capacity() / m_sample_bytes) * sizeof(int16_t);
    uint32_t fill_bytes   = m_fill * output_bytes;
    uint32_t buffer_bytes = output_bytes + fill_bytes;

    MFCreateMemoryBuffer(buffer_bytes, &pBuffer);

    pBuffer->Lock(&pDst, NULL, NULL);

    int16_t* high_addr = (int16_t*)(pDst + fill_bytes);

    m_kernel_cast(high_addr,      (float*)reference.data(), output_bytes);
    m_kernel_wide((int16_t*)pDst, high_addr,                output_bytes);

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
