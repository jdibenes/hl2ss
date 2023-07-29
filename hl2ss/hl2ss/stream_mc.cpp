
#include <mfapi.h>
#include "custom_media_sink.h"
#include "ports.h"
#include "microphone_capture.h"
#include "ipc_sc.h"
#include "log.h"
#include <chrono>

#include "zenoh.h"

#define FASTCDR_STATIC_LINK
#include "fastcdr/Cdr.h"

#include "pcpd_msgs/msg/Hololens2AACAudioStream.h"
#include "pcpd_msgs/msg/Hololens2Sensors.h"

struct MC_Context {
	std::string client_id;
	z_session_t session;
	AACFormat format;
	bool valid{ false };
	bool first_frame_sent{ false };
};


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_thread = NULL; // CloseHandle
static HANDLE g_event_quit = NULL; // CloseHandle

static winrt::com_ptr<MicrophoneCapture> g_microphoneCapture = nullptr;

static MC_Context* g_zenoh_context = NULL;

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
	eprosima::fastcdr::Cdr buffer_cdr(buffer);

	pcpd_msgs::msg::Hololens2AACAudioStream value{};

	{
		using namespace std::chrono;
		auto ts_ = nanoseconds(sampletime * 100);
		auto ts_sec = static_cast<int32_t>(duration_cast<seconds>(ts_).count());
		auto ts_nsec = static_cast<int32_t>(duration_cast<nanoseconds>(ts_ - seconds(ts_sec)).count());

		value.header().stamp().sec(ts_sec);
		value.header().stamp().nanosec(ts_nsec);

		value.header().frame_id(g_zenoh_context->client_id);
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
		ShowMessage("PV: Error publishing message");
		SetEvent(user->clientevent);
	}
	else {
		//ShowMessage("PV: published frame");
	}

	pBuffer->Unlock();
	pBuffer->Release();
}

// OK
static void MC_Shoutcast()
{

	if (g_zenoh_context == NULL || !g_zenoh_context->valid) {
		ShowMessage("PV: Error invalid context");
		return;
	}

	std::string keyexpr = "hl2/sensor/mic/" + g_zenoh_context->client_id;
	ShowMessage("MC: publish on: %s", keyexpr.c_str());

	z_owned_publisher_t pub = z_declare_publisher(g_zenoh_context->session, z_keyexpr(keyexpr.c_str()), NULL);

	if (!z_check(pub)) {
		ShowMessage("MC: Error creating publisher");
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

	AACFormat format = g_zenoh_context->format;

	format.channels   = channels;
	format.samplerate = samplerate;

	event_client = CreateEvent(NULL, TRUE, FALSE, NULL);

	user.publisher = z_loan(pub);
	user.clientevent  = event_client;
	user.data_profile = format.profile;
	user.options = options;
	user.client_id = g_zenoh_context->client_id.c_str();

	switch (format.profile)
	{
	case AACProfile::AACProfile_None: CreateSinkWriterPCMToPCM(&pSink, &pSinkWriter, &dwAudioIndex, format, MC_SendSampleToSocket, &user); break;
	default:                          CreateSinkWriterPCMToAAC(&pSink, &pSinkWriter, &dwAudioIndex, format, MC_SendSampleToSocket, &user); break;
	}

	if (!g_zenoh_context->first_frame_sent) {
		g_zenoh_context->first_frame_sent = true;

		pcpd_msgs::msg::Hololens2StreamDescriptor value{};

		value.stream_topic("hl2/sensor/mic/" + g_zenoh_context->client_id);
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
		eprosima::fastcdr::Cdr buffer_cdr(buffer);

		buffer_cdr.reset();
		value.serialize(buffer_cdr);

		// put message to zenoh
		std::string keyexpr1 = "hl2/cfg/mic/" + g_zenoh_context->client_id;
		z_put_options_t options1 = z_put_options_default();
		options1.encoding = z_encoding(Z_ENCODING_PREFIX_APP_CUSTOM, NULL);
		int res = z_put(g_zenoh_context->session, z_keyexpr(keyexpr1.c_str()), (const uint8_t*)buffer.getBuffer(), buffer_cdr.getSerializedDataLength(), &options1);
		if (res > 0) {
			ShowMessage("MC: Error putting info");
		}
		else {
			ShowMessage("MC: put info");
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

	ShowMessage("MC: Start MC Stream");

	MC_Shoutcast();

	//while (WaitForSingleObject(g_event_quit, 0) == WAIT_TIMEOUT);

	ShowMessage("MC: Finished MC Stream");

	return 0;
}

// OK
void MC_Initialize(const char* client_id, z_session_t session, AACFormat format)
{

	g_zenoh_context = new MC_Context(); // release
	g_zenoh_context->client_id = std::string(client_id);
	g_zenoh_context->session = session;
	g_zenoh_context->format = std::move(format);
	g_zenoh_context->valid = true;

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

	free(g_zenoh_context);
	g_zenoh_context = NULL;

}
