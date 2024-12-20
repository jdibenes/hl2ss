
#include "encoder_ea.h"

using namespace winrt::Windows::Media;
using namespace winrt::Windows::Media::MediaProperties;
using namespace winrt::Windows::Media::Capture::Frames;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

EA_AudioTransform const Encoder_EA::m_at_lut[4] =
{
    {F32ToS16, ExtendMonoToStereo, sizeof(float),   true},
    {F32ToS16, Bypass,             sizeof(float),   false},
    {S16ToS16, ExtendMonoToStereo, sizeof(int16_t), true},
    {S16ToS16, Bypass,             sizeof(int16_t), false}
};

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void Encoder_EA::AudioF32ToS16(int16_t* out, float const* in, int32_t samples)
{
	float32x4_t s = vdupq_n_f32(32767.0f);

	for (int i = 0; i < (samples / 16); ++i)
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
void Encoder_EA::AudioS16MonoToStereo(int16_t* out, int16_t const* in, int32_t samples)
{
	for (int i = 0; i < (samples / 32); ++i)
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
void Encoder_EA::F32ToS16(int16_t* out, void const* in, int32_t samples)
{
    AudioF32ToS16(out, static_cast<float const*>(in), samples);
}

// OK
void Encoder_EA::S16ToS16(int16_t* out, void const* in, int32_t samples)
{
    memcpy(out, in, samples);
}

// OK
void Encoder_EA::ExtendMonoToStereo(int16_t* out, int16_t const* in, int32_t samples)
{
    AudioS16MonoToStereo(out, in, samples);
}

// OK
void Encoder_EA::Bypass(int16_t* out, int16_t const* in, int32_t samples)
{
    (void)out;
    (void)in;
    (void)samples;
}

// OK
EA_AudioTransform Encoder_EA::GetTransform(AudioSubtype subtype, uint32_t channels)
{
    return m_at_lut[(2 * (subtype == AudioSubtype::AudioSubtype_S16)) + (channels == 2)];
}

// OK
void Encoder_EA::SetAACFormat(AACFormat& format)
{
    format.samplerate = 48000;
    format.channels   = 2;
}

// OK
Encoder_EA::Encoder_EA(HOOK_ENCODER_PROC pHookCallback, void* pHookParam, AudioSubtype subtype, AACFormat const& format, uint32_t channels) :
CustomEncoder(pHookCallback, pHookParam, NULL, 0, AudioSubtype::AudioSubtype_S16, format)
{
    m_kernel_cast  = GetTransform(subtype, channels).cast_kernel;
    m_kernel_wide  = GetTransform(subtype, channels).wide_kernel;
    m_sample_bytes = GetTransform(subtype, channels).sample_bytes;
    m_fill         = GetTransform(subtype, channels).fill;
}

// OK
void Encoder_EA::WriteSample(MediaFrameReference const& frame)
{
    IMFMediaBuffer* pBuffer; // Release
    BYTE* pDst;

    auto amf       = frame.AudioMediaFrame();
    auto audio     = amf.GetAudioFrame();
    auto buffer    = audio.LockBuffer(AudioBufferAccessMode::Read);
    auto reference = buffer.CreateReference();

    uint32_t samples      = reference.Capacity() / m_sample_bytes;
    uint32_t output_bytes = samples * sizeof(int16_t);
    uint32_t fill_bytes   = m_fill * output_bytes;
    uint32_t buffer_bytes = output_bytes + fill_bytes;

    CreateBuffer(&pBuffer, buffer_bytes);

    pBuffer->Lock(&pDst, NULL, NULL);

    void*    data_addr = reference.data();
    int16_t* base_addr = reinterpret_cast<int16_t*>(pDst);
    int16_t* high_addr = reinterpret_cast<int16_t*>(pDst + fill_bytes);

    m_kernel_cast(high_addr, data_addr, samples);
    m_kernel_wide(base_addr, high_addr, samples);

    pBuffer->Unlock();

    WriteBuffer(pBuffer, frame.SystemRelativeTime().Value().count(), frame.Duration().count(), NULL);

    pBuffer->Release();

    reference.Close();
    buffer.Close();
    audio.Close();
}
