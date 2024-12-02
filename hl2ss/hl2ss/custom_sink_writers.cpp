
#include <mfapi.h>
#include <codecapi.h>
#include "custom_sink_writers.h"

struct AVOption
{
	GUID guid;
	uint32_t vt;
};

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

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
static void TranslateEncoderOptions(std::vector<uint64_t> const& options, IMFAttributes **pEncoderAttr)
{
	size_t size = options.size() & ~1ULL;

	MFCreateAttributes(pEncoderAttr, static_cast<UINT32>(size / 2));

	for (int i = 0; i < (int)size; i += 2)
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

// OK
CustomSinkWriter::CustomSinkWriter(HOOK_SINK_PROC hookproc, void* hookparam, IMFMediaType* pInputType, IMFMediaType* pOutputType, IMFAttributes* pEncoderAttr)
{
	IMFAttributes* pSinkAttr; // Release

	MFCreateAttributes(&pSinkAttr, 3);
	
	pSinkAttr->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, TRUE);
	pSinkAttr->SetUINT32(MF_LOW_LATENCY, TRUE);
	pSinkAttr->SetUINT32(MF_SINK_WRITER_DISABLE_THROTTLING, TRUE);

	CustomMediaSink::CreateInstance(&m_pSink, MEDIASINK_RATELESS, hookproc, hookparam);
	MFCreateSinkWriterFromMediaSink(m_pSink, pSinkAttr, &m_pSinkWriter);

	m_pSinkWriter->AddStream(pOutputType, &m_dwStreamIndex);
	m_pSinkWriter->SetInputMediaType(m_dwStreamIndex, pInputType, pEncoderAttr);
	m_pSinkWriter->BeginWriting();

	pSinkAttr->Release();
}

// OK
CustomSinkWriter::~CustomSinkWriter()
{
	m_pSinkWriter->Finalize();
	m_pSinkWriter->Release();
	m_pSink->Shutdown();
	m_pSink->Release();
}

// OK
void CustomSinkWriter::WriteSample(IMFSample* pSample)
{
	m_pSinkWriter->WriteSample(m_dwStreamIndex, pSample);
}

// OK
std::unique_ptr<CustomSinkWriter> CustomSinkWriter::CreateForAudio(HOOK_SINK_PROC hookproc, void* hookparam, AudioSubtype input_subtype, AACFormat const& format)
{
	IMFMediaType* pTypeInput; // Release
	IMFMediaType* pTypeOutput; // Release

	CreateTypeAudio(&pTypeInput,  format.channels, format.samplerate, input_subtype, AACProfile::AACProfile_None, AACLevel::AACLevel_Default);
	CreateTypeAudio(&pTypeOutput, format.channels, format.samplerate, input_subtype, format.profile,              format.level);

	std::unique_ptr<CustomSinkWriter> pSinkWriter = std::make_unique<CustomSinkWriter>(hookproc, hookparam, pTypeInput, pTypeOutput, nullptr);

	pTypeInput->Release();
	pTypeOutput->Release();

	return pSinkWriter;
}

// OK
std::unique_ptr<CustomSinkWriter> CustomSinkWriter::CreateForVideo(HOOK_SINK_PROC hookproc, void* hookparam, VideoSubtype input_subtype, H26xFormat const& format, uint32_t stride, std::vector<uint64_t> const& encoder_options)
{
	IMFMediaType* pTypeInput; // Release
	IMFMediaType* pTypeOutput; // Release
	IMFAttributes* pEncoderAttr; // Release

	CreateTypeVideo(&pTypeInput,  format.width, format.height, stride, format.framerate, format.divisor, input_subtype, H26xProfile::H26xProfile_None, H26xLevel_Default, 0);
	CreateTypeVideo(&pTypeOutput, format.width, format.height, stride, format.framerate, format.divisor, input_subtype, format.profile,                format.level,      format.bitrate);

	TranslateEncoderOptions(encoder_options, &pEncoderAttr);

	std::unique_ptr<CustomSinkWriter> pSinkWriter = std::make_unique<CustomSinkWriter>(hookproc, hookparam, pTypeInput, pTypeOutput, pEncoderAttr);

	pTypeInput->Release();
	pTypeOutput->Release();
	pEncoderAttr->Release();

	return pSinkWriter;
}
