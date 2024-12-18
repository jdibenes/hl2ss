
#include "server.h"
#include "custom_encoder.h"

#include "extended_execution.h"
#include "holographic_space.h"
#include "locator.h"
#include "timestamp.h"

#include "extended_audio.h"
#include "extended_eye_tracking.h"
#include "extended_video.h"
#include "message_queue.h"
#include "microphone_capture.h"
#include "personal_video.h"
#include "research_mode.h"
#include "scene_understanding.h"
#include "spatial_input.h"
#include "spatial_mapping.h"
#include "voice_input.h"

#include "ipc_mq.h"
#include "ipc_mqx.h"
#include "ipc_rc.h"
#include "ipc_sm.h"
#include "ipc_su.h"
#include "ipc_vi.h"

#include "stream_ea.h"
#include "stream_eet.h"
#include "stream_ev.h"
#include "stream_mc.h"
#include "stream_pv.h"
#include "stream_rm_acc.h"
#include "stream_rm_gyr.h"
#include "stream_rm_mag.h"
#include "stream_rm_vlc.h"
#include "stream_rm_zht.h"
#include "stream_rm_zlt.h"
#include "stream_si.h"

#include "log.h"

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static bool g_standalone = false;
static bool g_flat = false;
static bool g_rm = false;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void HL2SS_Load(bool standalone)
{
	g_standalone = standalone;

	Server_Startup();
	CustomEncoder_Startup();

	if (g_standalone)
	{
	ExtendedExecution_Initialize();
	g_flat = ExtendedExecution_GetFlatMode();
	if (!g_flat)
	{
	HolographicSpace_Initialize();
	}
	}

	Locator_Initialize();
	Timestamp_Initialize();

	ResearchMode_Startup();
	PersonalVideo_Startup();
	MicrophoneCapture_Startup();
	SpatialInput_Startup();
	// - EV
	// - EA
	// - EET
	// - SM
	SceneUnderstanding_Startup();
	VoiceInput_Startup();
	MessageQueue_Server_Startup();
	MessageQueue_Client_Startup();

	g_rm = ResearchMode_Status();

	if (g_rm)
	{
	RM_VLF_Startup();
	RM_VLL_Startup();
	RM_VRF_Startup();
	RM_VRR_Startup();
	RM_ZHT_Startup();
	RM_ZLT_Startup();
	RM_ACC_Startup();
	RM_GYR_Startup();
	RM_MAG_Startup();
	}

	PV_Startup();
	MC_Startup();
	SI_Startup();
	EV_Startup();
	EA_Startup();
	EET_Startup();

	RC_Startup();
	SM_Startup();
	SU_Startup();
	VI_Startup();
	MQ_Startup();
	MQX_Startup();
}

// OK
void HL2SS_Process_HS()
{
	if (g_flat) { return; }

	HolographicSpace_Update();
	HolographicSpace_Clear();
	// Draw
	HolographicSpace_Present();
}

// OK
void HL2SS_Process_MQ()
{
	uint32_t size = MessageQueue_Server_RX_Peek();
	if (size == MQ_MARKER) { return; }

	uint32_t command;
	char* data = new char[size + 1]; // delete[]
	MessageQueue_Server_RX_Pull(command, data);
	data[size] = '\0';

	switch (command)
	{
	case 0xFFFFFFFE:
		ShowMessage("MQ: %s", data);
		MessageQueue_Server_TX_Push(0);
		break;
	case MQ_MARKER:
		MessageQueue_Server_Restart();
		break;
	default:
		ShowMessage("MQ: Received command %x with size %d", command, size);
		MessageQueue_Server_TX_Push(MQ_MARKER);
		break;
	}

	delete[] data;
}

// OK
void HL2SS_Process_MQX()
{
	char const* const text = "Hello from HoloLens 2";

	static int state = 0;
	uint32_t id;

	switch (state)
	{
	case 0:
		MessageQueue_Client_TX_Push(0xFFFFFFFE, static_cast<uint32_t>(strlen(text)), text);
		state = 1;
		break;
	case 1:
		if (MessageQueue_Client_RX_Peek() == MQ_MARKER) { break; }
		MessageQueue_Client_RX_Pull(id);
		if (id == MQ_MARKER) { MessageQueue_Client_Restart(); } else { ShowMessage("MQX: Received response %x", id); }
		state = 0;
		break;
	}
}
