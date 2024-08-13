
#include <mfapi.h>
#include <codecapi.h>
#include "server.h"
#include "custom_media_sink.h"
#include "custom_media_types.h"
#include "custom_hook_callback.h"
#include "custom_sink_writers.h"

struct AVOption
{
	GUID guid;
	uint32_t vt;
};

static AVOption g_AVLUT[] =
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

// OK
static uint32_t ComputeStride(uint32_t width)
{
	uint32_t const align = 64;
	uint32_t const mask = align - 1;

	return (width + mask) & ~mask;
}

// OK
static void TranslateEncoderOptions(std::vector<uint64_t> const& options, IMFAttributes **pEncoderAttr)
{
	size_t size = options.size() & ~1ULL;

	MFCreateAttributes(pEncoderAttr, (UINT32)size);

	for (int i = 0; i < (int)size; i += 2)
	{
	uint64_t option = options[i];
	uint64_t value  = options[i + 1];

	if (option >= (sizeof(g_AVLUT) / sizeof(AVOption))) { continue; }

	AVOption entry = g_AVLUT[option];

	switch (entry.vt)
	{
	case VT_UI4:  (*pEncoderAttr)->SetUINT32(entry.guid, (UINT32)value); break;
	case VT_UI8:  (*pEncoderAttr)->SetUINT64(entry.guid, value); break;
	case VT_BOOL: (*pEncoderAttr)->SetUINT32(entry.guid, (value == 0) ? VARIANT_FALSE : VARIANT_TRUE); break;
	}
	}
}

// OK
static void CreateSingleStreamSinkWriter(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwStreamIndex, IMFMediaType* pInputType, IMFMediaType* pOutputType, IMFAttributes* pEncoderAttr, HOOK_SINK_PROC hookproc, void* hookparam)
{
	HookSinkCallback* pHook; // Release
	CustomMediaSink* pSink;
	IMFAttributes* pSinkAttr; // Release	
	IMFSinkWriter* pSinkWriter;
	DWORD dwStreamIndex;

	HookSinkCallback::CreateInstance(&pHook, hookproc, hookparam);
	CustomMediaSink::CreateInstance(&pSink, MEDIASINK_RATELESS, pHook);

	MFCreateAttributes(&pSinkAttr, 3);
	
	pSinkAttr->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, TRUE);
	pSinkAttr->SetUINT32(MF_LOW_LATENCY, TRUE);
	pSinkAttr->SetUINT32(MF_SINK_WRITER_DISABLE_THROTTLING, TRUE);

	MFCreateSinkWriterFromMediaSink(pSink, pSinkAttr, &pSinkWriter);

	pSinkWriter->AddStream(pOutputType, &dwStreamIndex);
	pSinkWriter->SetInputMediaType(dwStreamIndex, pInputType, pEncoderAttr);
	pSinkWriter->BeginWriting();

	*ppSink = pSink;
	*ppSinkWriter = pSinkWriter;
	*pdwStreamIndex = dwStreamIndex;

	pHook->Release();
	pSinkAttr->Release();
}

// OK
void CreateSinkWriterAudio(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwAudioIndex, AudioSubtype input_subtype, AACFormat const& format, HOOK_SINK_PROC hookproc, void* hookparam)
{
	IMFMediaType* pTypeInput; // Release
	IMFMediaType* pTypeOutput; // Release

	CreateTypeAudio(&pTypeInput, format.channels, format.samplerate, input_subtype, AACProfile::AACProfile_None, AACLevel::AACLevel_Default);
	CreateTypeAudio(&pTypeOutput, format.channels, format.samplerate, input_subtype, format.profile, format.level);

	CreateSingleStreamSinkWriter(ppSink, ppSinkWriter, pdwAudioIndex, pTypeInput, pTypeOutput, NULL, hookproc, hookparam);

	pTypeInput->Release();
	pTypeOutput->Release();
}

// OK
void CreateSinkWriterVideo(CustomMediaSink** ppSink, IMFSinkWriter** ppSinkWriter, DWORD* pdwVideoIndex, VideoSubtype input_subtype, H26xFormat const& format, std::vector<uint64_t> const& encoder_options, HOOK_SINK_PROC hookproc, void* hookparam)
{
	uint32_t stride = ComputeStride(format.width);

	IMFMediaType* pTypeInput; // Release
	IMFMediaType* pTypeOutput; // Release
	IMFAttributes* pEncoderAttr; // Release

	CreateTypeVideo(&pTypeInput, format.width, format.height, stride, format.framerate, format.divisor, input_subtype, H26xProfile::H26xProfile_None, H26xLevel_Default, 0);
	CreateTypeVideo(&pTypeOutput, format.width, format.height, stride, format.framerate, format.divisor, input_subtype, format.profile, format.level, format.bitrate);

	TranslateEncoderOptions(encoder_options, &pEncoderAttr);

	CreateSingleStreamSinkWriter(ppSink, ppSinkWriter, pdwVideoIndex, pTypeInput, pTypeOutput, pEncoderAttr, hookproc, hookparam);

	pTypeInput->Release();
	pTypeOutput->Release();
	pEncoderAttr->Release();
}
