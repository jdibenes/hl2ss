
#include <mfapi.h>
#include "custom_media_sink.h"
#include "ports.h"
#include "microphone_capture.h"
#include "ipc_sc.h"
#include "log.h"
#include <chrono>

#include "hl2ss_network.h"

#define FASTCDR_STATIC_LINK
#include "fastcdr/Cdr.h"

#include "pcpd_msgs/msg/Hololens2AudioStream.h"
#include "pcpd_msgs/msg/Hololens2Sensors.h"


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread = NULL; // CloseHandle
static HANDLE g_event_quit = NULL; // CloseHandle

static winrt::com_ptr<MicrophoneCapture> g_microphoneCapture = nullptr;

static HC_Context_Ptr g_zenoh_context;
static AACFormat g_aac_format = AACFormat{};
static bool g_first_frame_sent = false;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void MC_SendSampleToSocket(IMFSample* pSample, void* param)
{
	IMFMediaBuffer* pBuffer; // Release
	LONGLONG sampletime;
	BYTE* pBytes;
	DWORD cbData;
	HookCallbackSocket* user;

	user = (HookCallbackSocket*)param;

	pSample->GetSampleTime(&sampletime);
	pSample->ConvertToContiguousBuffer(&pBuffer);

	pBuffer->Lock(&pBytes, NULL, &cbData);

	// serialization

 // can we cache them so that we do not allocate new memory every image ?
	eprosima::fastcdr::FastBuffer buffer{};
	eprosima::fastcdr::Cdr buffer_cdr(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);

	pcpd_msgs::msg::Hololens2AudioStream value{};

	{
		using namespace std::chrono;
		auto ts_ = nanoseconds(sampletime * 100);
		auto ts_sec = static_cast<int32_t>(duration_cast<seconds>(ts_).count());
		auto ts_nsec = static_cast<int32_t>(duration_cast<nanoseconds>(ts_ - seconds(ts_sec)).count());

		value.header().stamp().sec(ts_sec);
		value.header().stamp().nanosec(ts_nsec);

		value.header().frame_id(g_zenoh_context->topic_prefix);
	}

	// format
	value.data_size(cbData);

	// this copies the buffer .. is there another way?
	std::vector<uint8_t> bsbuf(cbData);
	bsbuf.assign(pBytes, pBytes + cbData);
	value.data(std::move(bsbuf));


	buffer_cdr.reset();
	value.serialize(buffer_cdr);

	if (z_publisher_put(user->publisher, (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &(user->options))) {
		SPDLOG_INFO("PV: Error publishing message");
		SetEvent(user->clientevent);
	}
	else {
		//SPDLOG_INFO("PV: published frame");
	}

	pBuffer->Unlock();
	pBuffer->Release();
}

// OK
static void MC_Shoutcast()
{

	if (!g_zenoh_context || !g_zenoh_context->valid) {
		SPDLOG_INFO("PV: Error invalid context");
		return;
	}

	std::string keyexpr = g_zenoh_context->topic_prefix + "/str/mic";
	SPDLOG_INFO("MC: publish on: {0}", keyexpr.c_str());

	z_publisher_options_t publisher_options = z_publisher_options_default();
	publisher_options.priority = Z_PRIORITY_REAL_TIME;

	z_owned_publisher_t pub = z_declare_publisher(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr.c_str()), &publisher_options);

	if (!z_check(pub)) {
		SPDLOG_INFO("MC: Error creating publisher");
		return;
	}

	z_publisher_put_options_t options = z_publisher_put_options_default();
	options.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);


	uint32_t const channels   = 2;
	uint32_t const samplerate = 48000;
	
	CustomMediaSink* pSink; // Release
	IMFSinkWriter* pSinkWriter; // Release
	HANDLE event_client; // CloseHandle
	HookCallbackSocket user;
	DWORD dwAudioIndex;

	AACFormat format = g_aac_format;

	format.channels   = channels;
	format.samplerate = samplerate;

	event_client = CreateEvent(NULL, TRUE, FALSE, NULL);

	user.publisher = z_loan(pub);
	user.clientevent  = event_client;
	user.data_profile = format.profile;
	user.options = options;
	user.topic_prefix = g_zenoh_context->topic_prefix.c_str();

	switch (format.profile)
	{
	case AACProfile::AACProfile_None: CreateSinkWriterPCMToPCM(&pSink, &pSinkWriter, &dwAudioIndex, format, MC_SendSampleToSocket, &user); break;
	default:                          CreateSinkWriterPCMToAAC(&pSink, &pSinkWriter, &dwAudioIndex, format, MC_SendSampleToSocket, &user); break;
	}

	if (!g_first_frame_sent) {
		g_first_frame_sent = true;

		pcpd_msgs::msg::Hololens2StreamDescriptor value{};

		value.stream_topic(g_zenoh_context->topic_prefix + "/str/mic");
		value.sensor_type(pcpd_msgs::msg::Hololens2SensorType::MICROPHONE);
		value.frame_rate(format.samplerate);

		switch (format.profile) {
		case AACProfile_None:
			value.aac_profile(pcpd_msgs::msg::AACProfile_None);
			break;
		case AACProfile_12000:
			value.aac_profile(pcpd_msgs::msg::AACProfile_12000);
			break;
		case AACProfile_16000:
			value.aac_profile(pcpd_msgs::msg::AACProfile_16000);
			break;
		case AACProfile_20000:
			value.aac_profile(pcpd_msgs::msg::AACProfile_20000);
			break;
		case AACProfile_24000:
			value.aac_profile(pcpd_msgs::msg::AACProfile_24000);
			break;
		}

		value.audio_channels(static_cast<uint8_t>(format.channels));


		eprosima::fastcdr::FastBuffer buffer{};
		eprosima::fastcdr::Cdr buffer_cdr(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::Cdr::DDS_CDR);

		buffer_cdr.reset();
		value.serialize(buffer_cdr);

		// put message to zenoh
		std::string keyexpr1 = g_zenoh_context->topic_prefix + "/cfg/desc/mic";
		z_put_options_t options1 = z_put_options_default();
		options1.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
		int res = z_put(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr1.c_str()), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options1);
		if (res > 0) {
			SPDLOG_INFO("MC: Error putting info");
		}
		else {
			SPDLOG_INFO("MC: put info: {}", keyexpr1);
		}

	}

	g_microphoneCapture->Start();
	do { g_microphoneCapture->WriteSample(pSinkWriter, dwAudioIndex); } while (WaitForSingleObject(event_client, 0) == WAIT_TIMEOUT);
	g_microphoneCapture->Stop();
	
	pSinkWriter->Flush(dwAudioIndex);
	pSinkWriter->Release();
	pSink->Shutdown();
	pSink->Release();


	z_undeclare_publisher(z_move(pub));

	CloseHandle(event_client);
}

// OK
static DWORD WINAPI MC_EntryPoint(void*)
{

	g_microphoneCapture->WaitActivate(INFINITE);

	SPDLOG_INFO("MC: Start MC Stream");

	MC_Shoutcast();

	//while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);

	SPDLOG_INFO("MC: Finished MC Stream");

	return 0;
}

// OK
void MC_Initialize(HC_Context_Ptr& context, AACFormat format)
{

	g_zenoh_context = context;
	g_aac_format = std::move(format);

	g_microphoneCapture = winrt::make_self<MicrophoneCapture>();
	g_microphoneCapture->Initialize();

	g_event_quit = CreateEvent(NULL, TRUE, FALSE, NULL);
	g_thread = CreateThread(NULL, 0, MC_EntryPoint, NULL, 0, NULL);
}

// OK
void MC_Quit()
{
	SetEvent(g_event_quit);
}

// OK
void MC_Cleanup()
{
	WaitForSingleObject(g_thread, INFINITE);

	CloseHandle(g_thread);
	CloseHandle(g_event_quit);

	g_thread = NULL;
	g_event_quit = NULL;
	g_microphoneCapture = nullptr;

	g_zenoh_context.reset();
    g_first_frame_sent = false;

}
