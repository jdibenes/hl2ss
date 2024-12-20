
#include <mfapi.h>
#include <codecapi.h>
#include "custom_media_types.h"

struct AVOption
{
    GUID guid;
    uint32_t vt;
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static AVOption const g_AVLUT[] =
{
    {CODECAPI_AVEncCommonRateControlMode, VT_UI4},
    {CODECAPI_AVEncCommonQuality, VT_UI4},
    {CODECAPI_AVEncAdaptiveMode, VT_UI4},
    {CODECAPI_AVEncCommonBufferSize, VT_UI4},
    {CODECAPI_AVEncCommonMaxBitRate, VT_UI4},
    {CODECAPI_AVEncCommonMeanBitRate, VT_UI4},
    {CODECAPI_AVEncCommonQualityVsSpeed, VT_UI4},
    {CODECAPI_AVEncH264CABACEnable, VT_BOOL},
    {CODECAPI_AVEncH264SPSID, VT_UI4},
    {CODECAPI_AVEncMPVDefaultBPictureCount, VT_UI4},
    {CODECAPI_AVEncMPVGOPSize, VT_UI4},
    {CODECAPI_AVEncNumWorkerThreads, VT_UI4},
    {CODECAPI_AVEncVideoContentType, VT_UI4},
    {CODECAPI_AVEncVideoEncodeQP, VT_UI8},
    {CODECAPI_AVEncVideoForceKeyFrame, VT_UI4},
    {CODECAPI_AVEncVideoMinQP, VT_UI4},
    {CODECAPI_AVLowLatencyMode, VT_BOOL},
    {CODECAPI_AVEncVideoMaxQP, VT_UI4},
    {CODECAPI_VideoEncoderDisplayContentType, VT_UI4},
};

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// https://learn.microsoft.com/en-us/windows/win32/medfound/audio-subtype-guids
// OK
static HRESULT CreateTypePCMF32(IMFMediaType** ppType, uint32_t channels, uint32_t samplerate)
{
    uint32_t const bitspersample = 32;

    uint32_t blockalign = channels * (bitspersample / 8);
    uint32_t bytespersecond = blockalign * samplerate;
    IMFMediaType* pType;

    MFCreateMediaType(&pType);

    pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Audio);
    pType->SetGUID(MF_MT_SUBTYPE, MFAudioFormat_Float);
    pType->SetUINT32(MF_MT_AUDIO_NUM_CHANNELS, channels);
    pType->SetUINT32(MF_MT_AUDIO_SAMPLES_PER_SECOND, samplerate);
    pType->SetUINT32(MF_MT_AUDIO_BLOCK_ALIGNMENT, blockalign);
    pType->SetUINT32(MF_MT_AUDIO_AVG_BYTES_PER_SECOND, bytespersecond);
    pType->SetUINT32(MF_MT_AUDIO_BITS_PER_SAMPLE, bitspersample);
    pType->SetUINT32(MF_MT_ALL_SAMPLES_INDEPENDENT, TRUE);

    *ppType = pType;

    return S_OK;
}

// https://learn.microsoft.com/en-us/windows/win32/medfound/audio-subtype-guids
// OK
static HRESULT CreateTypePCMS16(IMFMediaType** ppType, uint32_t channels, uint32_t samplerate)
{
    uint32_t const bitspersample = 16;

    uint32_t blockalign = channels * (bitspersample / 8);
    uint32_t bytespersecond = blockalign * samplerate;
    IMFMediaType* pType;

    MFCreateMediaType(&pType);

    pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Audio);
    pType->SetGUID(MF_MT_SUBTYPE, MFAudioFormat_PCM);
    pType->SetUINT32(MF_MT_AUDIO_NUM_CHANNELS, channels);
    pType->SetUINT32(MF_MT_AUDIO_SAMPLES_PER_SECOND, samplerate);
    pType->SetUINT32(MF_MT_AUDIO_BLOCK_ALIGNMENT, blockalign);
    pType->SetUINT32(MF_MT_AUDIO_AVG_BYTES_PER_SECOND, bytespersecond);
    pType->SetUINT32(MF_MT_AUDIO_BITS_PER_SAMPLE, bitspersample);
    pType->SetUINT32(MF_MT_ALL_SAMPLES_INDEPENDENT, TRUE);

    *ppType = pType;

    return S_OK;
}

// https://docs.microsoft.com/en-us/windows/win32/medfound/aac-encoder
// OK
static HRESULT CreateTypeAAC(IMFMediaType** ppType, uint32_t channels, uint32_t samplerate, uint32_t bytespersecond, uint32_t level)
{
    IMFMediaType* pType;
    
    MFCreateMediaType(&pType);

    pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Audio);
    pType->SetGUID(MF_MT_SUBTYPE, MFAudioFormat_AAC);
    pType->SetUINT32(MF_MT_AUDIO_BITS_PER_SAMPLE, 16);
    pType->SetUINT32(MF_MT_AUDIO_NUM_CHANNELS, channels);
    pType->SetUINT32(MF_MT_AUDIO_SAMPLES_PER_SECOND, samplerate);
    pType->SetUINT32(MF_MT_AUDIO_AVG_BYTES_PER_SECOND, bytespersecond);
    pType->SetUINT32(MF_MT_AAC_PAYLOAD_TYPE, 1);
    pType->SetUINT32(MF_MT_AAC_AUDIO_PROFILE_LEVEL_INDICATION, level);

    *ppType = pType;

    return S_OK;
}

// OK
HRESULT CreateTypeAudio(IMFMediaType** ppType, uint32_t channels, uint32_t samplerate, AudioSubtype subtype, AACProfile profile, AACLevel level)
{
    switch (profile)
    {
    case AACProfile::AACProfile_12000: return CreateTypeAAC(ppType, channels, samplerate, 12000, level);
    case AACProfile::AACProfile_16000: return CreateTypeAAC(ppType, channels, samplerate, 16000, level);
    case AACProfile::AACProfile_20000: return CreateTypeAAC(ppType, channels, samplerate, 20000, level);
    case AACProfile::AACProfile_24000: return CreateTypeAAC(ppType, channels, samplerate, 24000, level);
    }

    switch (subtype)
    {
    case AudioSubtype::AudioSubtype_F32: return CreateTypePCMF32(ppType, channels, samplerate);
    case AudioSubtype::AudioSubtype_S16: return CreateTypePCMS16(ppType, channels, samplerate);
    }

    *ppType = NULL;

    return E_INVALIDARG;
}

// https://learn.microsoft.com/en-us/windows/win32/medfound/video-subtype-guids
// OK
static HRESULT CreateTypeL8(IMFMediaType **ppType, uint32_t width, uint32_t height, uint32_t stride, uint32_t fps_num, uint32_t fps_den)
{
    IMFMediaType* pType;

    MFCreateMediaType(&pType);

    pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    pType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_L8);
    pType->SetUINT32(MF_MT_DEFAULT_STRIDE, stride);
    MFSetAttributeRatio(pType, MF_MT_FRAME_RATE, fps_num, fps_den);
    MFSetAttributeSize(pType, MF_MT_FRAME_SIZE, width, height);
    pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlaceMode::MFVideoInterlace_Progressive);
    pType->SetUINT32(MF_MT_ALL_SAMPLES_INDEPENDENT, TRUE);
    MFSetAttributeRatio(pType, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    pType->SetUINT32(MF_MT_SAMPLE_SIZE, width * height);
    pType->SetUINT32(MF_MT_FIXED_SIZE_SAMPLES, TRUE);

    *ppType = pType;

    return S_OK;
}

// https://learn.microsoft.com/en-us/windows/win32/medfound/video-subtype-guids
// OK
static HRESULT CreateTypeL16(IMFMediaType **ppType, uint32_t width, uint32_t height, uint32_t stride, uint32_t fps_num, uint32_t fps_den)
{
    IMFMediaType* pType;

    MFCreateMediaType(&pType);

    pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    pType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_L16);
    pType->SetUINT32(MF_MT_DEFAULT_STRIDE, stride);
    MFSetAttributeRatio(pType, MF_MT_FRAME_RATE, fps_num, fps_den);
    MFSetAttributeSize(pType, MF_MT_FRAME_SIZE, width, height);
    pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlaceMode::MFVideoInterlace_Progressive);
    pType->SetUINT32(MF_MT_ALL_SAMPLES_INDEPENDENT, TRUE);
    MFSetAttributeRatio(pType, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    pType->SetUINT32(MF_MT_SAMPLE_SIZE, width * height * 2);
    pType->SetUINT32(MF_MT_FIXED_SIZE_SAMPLES, TRUE);

    *ppType = pType;

    return S_OK;
}

// https://learn.microsoft.com/en-us/windows/win32/medfound/video-subtype-guids
// OK
static HRESULT CreateTypeNV12(IMFMediaType **ppType, uint32_t width, uint32_t height, uint32_t stride, uint32_t fps_num, uint32_t fps_den)
{
    IMFMediaType* pType;

    MFCreateMediaType(&pType);

    pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    pType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_NV12);
    pType->SetUINT32(MF_MT_DEFAULT_STRIDE, stride);
    MFSetAttributeRatio(pType, MF_MT_FRAME_RATE, fps_num, fps_den);
    MFSetAttributeSize(pType, MF_MT_FRAME_SIZE, width, height);
    pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlaceMode::MFVideoInterlace_Progressive);
    pType->SetUINT32(MF_MT_ALL_SAMPLES_INDEPENDENT, TRUE);
    MFSetAttributeRatio(pType, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    pType->SetUINT32(MF_MT_SAMPLE_SIZE, (3UL * width * height) / 2UL);
    pType->SetUINT32(MF_MT_FIXED_SIZE_SAMPLES, TRUE);

    *ppType = pType;

    return S_OK;
}

// https://learn.microsoft.com/en-us/windows/win32/medfound/video-subtype-guids
// OK
static HRESULT CreateTypeARGB(IMFMediaType** ppType, uint32_t width, uint32_t height, uint32_t stride, uint32_t fps_num, uint32_t fps_den)
{
    IMFMediaType* pType;

    MFCreateMediaType(&pType);

    pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    pType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_ARGB32);
    pType->SetUINT32(MF_MT_DEFAULT_STRIDE, stride);
    MFSetAttributeRatio(pType, MF_MT_FRAME_RATE, fps_num, fps_den);
    MFSetAttributeSize(pType, MF_MT_FRAME_SIZE, width, height);
    pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlaceMode::MFVideoInterlace_Progressive);
    pType->SetUINT32(MF_MT_ALL_SAMPLES_INDEPENDENT, TRUE);
    MFSetAttributeRatio(pType, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    pType->SetUINT32(MF_MT_SAMPLE_SIZE, width * height * 4);
    pType->SetUINT32(MF_MT_FIXED_SIZE_SAMPLES, TRUE);

    *ppType = pType;

    return S_OK;
}

// https://learn.microsoft.com/en-us/windows/win32/medfound/video-subtype-guids
// OK
static HRESULT CreateTypeYUY2(IMFMediaType** ppType, uint32_t width, uint32_t height, uint32_t stride, uint32_t fps_num, uint32_t fps_den)
{
    IMFMediaType* pType;

    MFCreateMediaType(&pType);

    pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    pType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_YUY2);
    pType->SetUINT32(MF_MT_DEFAULT_STRIDE, stride);
    MFSetAttributeRatio(pType, MF_MT_FRAME_RATE, fps_num, fps_den);
    MFSetAttributeSize(pType, MF_MT_FRAME_SIZE, width, height);
    pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlaceMode::MFVideoInterlace_Progressive);
    pType->SetUINT32(MF_MT_ALL_SAMPLES_INDEPENDENT, TRUE);
    MFSetAttributeRatio(pType, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    pType->SetUINT32(MF_MT_SAMPLE_SIZE, 2UL * width * height);
    pType->SetUINT32(MF_MT_FIXED_SIZE_SAMPLES, TRUE);

    *ppType = pType;

    return S_OK;
}

// https://learn.microsoft.com/en-us/windows/win32/medfound/video-subtype-guids
// OK
static HRESULT CreateTypeIYUV(IMFMediaType** ppType, uint32_t width, uint32_t height, uint32_t stride, uint32_t fps_num, uint32_t fps_den)
{
    IMFMediaType* pType;

    MFCreateMediaType(&pType);

    pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    pType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_IYUV);
    pType->SetUINT32(MF_MT_DEFAULT_STRIDE, stride);
    MFSetAttributeRatio(pType, MF_MT_FRAME_RATE, fps_num, fps_den);
    MFSetAttributeSize(pType, MF_MT_FRAME_SIZE, width, height);
    pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlaceMode::MFVideoInterlace_Progressive);
    pType->SetUINT32(MF_MT_ALL_SAMPLES_INDEPENDENT, TRUE);
    MFSetAttributeRatio(pType, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    pType->SetUINT32(MF_MT_SAMPLE_SIZE, (3UL * width * height) / 2UL);
    pType->SetUINT32(MF_MT_FIXED_SIZE_SAMPLES, TRUE);

    *ppType = pType;

    return S_OK;
}

// https://learn.microsoft.com/en-us/windows/win32/medfound/video-subtype-guids
// OK
static HRESULT CreateTypeYV12(IMFMediaType** ppType, uint32_t width, uint32_t height, uint32_t stride, uint32_t fps_num, uint32_t fps_den)
{
    IMFMediaType* pType;

    MFCreateMediaType(&pType);

    pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    pType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_YV12);
    pType->SetUINT32(MF_MT_DEFAULT_STRIDE, stride);
    MFSetAttributeRatio(pType, MF_MT_FRAME_RATE, fps_num, fps_den);
    MFSetAttributeSize(pType, MF_MT_FRAME_SIZE, width, height);
    pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlaceMode::MFVideoInterlace_Progressive);
    pType->SetUINT32(MF_MT_ALL_SAMPLES_INDEPENDENT, TRUE);
    MFSetAttributeRatio(pType, MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    pType->SetUINT32(MF_MT_SAMPLE_SIZE, (3UL * width * height) / 2UL);
    pType->SetUINT32(MF_MT_FIXED_SIZE_SAMPLES, TRUE);

    *ppType = pType;

    return S_OK;
}

// https://docs.microsoft.com/en-us/windows/win32/medfound/h-264-video-encoder
// OK
static HRESULT CreateTypeH264(IMFMediaType** ppType, uint32_t width, uint32_t height, uint32_t fps_num, uint32_t fps_den, eAVEncH264VProfile profile, int32_t level, uint32_t bitrate)
{
    IMFMediaType* pType;

    MFCreateMediaType(&pType);

    pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    pType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_H264);
    pType->SetUINT32(MF_MT_AVG_BITRATE, bitrate);
    MFSetAttributeRatio(pType, MF_MT_FRAME_RATE, fps_num, fps_den);
    MFSetAttributeSize(pType, MF_MT_FRAME_SIZE, width, height);
    pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlaceMode::MFVideoInterlace_Progressive);
    pType->SetUINT32(MF_MT_MPEG2_PROFILE, profile);
    if (level != H26xLevel_Default) { pType->SetUINT32(MF_MT_MPEG2_LEVEL, level); }

    *ppType = pType;

    return S_OK;
}

// https://docs.microsoft.com/en-us/windows/win32/medfound/h-265---hevc-video-encoder
// OK
static HRESULT CreateTypeHEVC(IMFMediaType** ppType, uint32_t width, uint32_t height, uint32_t fps_num, uint32_t fps_den, eAVEncH265VProfile profile, int32_t level, uint32_t bitrate)
{
    IMFMediaType* pType;

    MFCreateMediaType(&pType);

    pType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    pType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_HEVC);
    pType->SetUINT32(MF_MT_AVG_BITRATE, bitrate);
    MFSetAttributeRatio(pType, MF_MT_FRAME_RATE, fps_num, fps_den);
    MFSetAttributeSize(pType, MF_MT_FRAME_SIZE, width, height);
    pType->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlaceMode::MFVideoInterlace_Progressive);
    pType->SetUINT32(MF_MT_MPEG2_PROFILE, profile);
    if (level != H26xLevel_Default) { pType->SetUINT32(MF_MT_MPEG2_LEVEL, level); }

    *ppType = pType;

    return S_OK;
}

// OK
HRESULT CreateTypeVideo(IMFMediaType** ppType, uint32_t width, uint32_t height, uint32_t stride, uint32_t fps_num, uint32_t fps_den, VideoSubtype subtype, H26xProfile profile, int32_t level, uint32_t bitrate)
{
    switch (profile)
    {
    case H26xProfile::H264Profile_Base: return CreateTypeH264(ppType, width, height, fps_num, fps_den, eAVEncH264VProfile::eAVEncH264VProfile_Base,       level, bitrate);
    case H26xProfile::H264Profile_Main: return CreateTypeH264(ppType, width, height, fps_num, fps_den, eAVEncH264VProfile::eAVEncH264VProfile_Main,       level, bitrate);
    case H26xProfile::H264Profile_High: return CreateTypeH264(ppType, width, height, fps_num, fps_den, eAVEncH264VProfile::eAVEncH264VProfile_High,       level, bitrate);
    case H26xProfile::H265Profile_Main: return CreateTypeHEVC(ppType, width, height, fps_num, fps_den, eAVEncH265VProfile::eAVEncH265VProfile_Main_420_8, level, bitrate);
    }

    switch (subtype)
    {
    case VideoSubtype::VideoSubtype_L8:   return CreateTypeL8(  ppType, width, height, stride, fps_num, fps_den);
    case VideoSubtype::VideoSubtype_L16:  return CreateTypeL16( ppType, width, height, stride, fps_num, fps_den);
    case VideoSubtype::VideoSubtype_NV12: return CreateTypeNV12(ppType, width, height, stride, fps_num, fps_den);
    case VideoSubtype::VideoSubtype_ARGB: return CreateTypeARGB(ppType, width, height, stride, fps_num, fps_den);
    case VideoSubtype::VideoSubtype_YUY2: return CreateTypeYUY2(ppType, width, height, stride, fps_num, fps_den);
    case VideoSubtype::VideoSubtype_IYUV: return CreateTypeIYUV(ppType, width, height, stride, fps_num, fps_den);
    case VideoSubtype::VideoSubtype_YV12: return CreateTypeYV12(ppType, width, height, stride, fps_num, fps_den);
    }

    *ppType = NULL;

    return E_INVALIDARG;
}

// OK
void TranslateEncoderOptions(std::vector<uint64_t> const& options, IMFAttributes **pEncoderAttr)
{
	size_t size = options.size() & ~1ULL;

	MFCreateAttributes(pEncoderAttr, static_cast<UINT32>(size / 2));

	for (int i = 0; i < static_cast<int>(size); i += 2)
	{
	uint64_t option = options[i];
	uint64_t value  = options[i + 1];

	if (option >= (sizeof(g_AVLUT) / sizeof(AVOption))) { continue; }

	AVOption entry = g_AVLUT[option];

	switch (entry.vt)
	{
	case VT_UI4:  (*pEncoderAttr)->SetUINT32(entry.guid, static_cast<UINT32>(value));                  break;
	case VT_UI8:  (*pEncoderAttr)->SetUINT64(entry.guid, value);                                       break;
	case VT_BOOL: (*pEncoderAttr)->SetUINT32(entry.guid, (value == 0) ? VARIANT_FALSE : VARIANT_TRUE); break;
	}
	}
}
