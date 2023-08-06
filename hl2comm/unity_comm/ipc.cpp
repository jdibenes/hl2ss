
#include <queue>
#include <malloc.h>
#include "plugin.h"

#include <spdlog/spdlog.h>
#include <zenoh.h>
#include "../hl2comm/lock.h"
#include "../hl2comm/log.h"
#include "../hl2comm/hl2ss_network.h"


//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static HANDLE g_event_quit; // CloseHandle
static CRITICAL_SECTION g_lock_sub; // DeleteCriticalSection


static HC_Context_Ptr g_zenoh_context{};

typedef void(*ZenohSubscriptionCallBack)(const z_sample_t* Sample);


struct subscriber_handle {
	const char* name{ nullptr };
	z_owned_keyexpr_t keyexpr{};
	z_subscriber_options_t options{};
	z_owned_subscriber_t subscriber{};

};

static std::vector<subscriber_handle> g_subscribers{};


struct subscriber_context {
	const char* name{ nullptr };
	ZenohSubscriptionCallBack callback;
};

void handle_subscriber_callback(const z_sample_t* sample, void* context) {
	auto handle = static_cast<subscriber_context*>(context);
	if (handle != nullptr && sample != nullptr) {
		try {
			handle->callback(sample);
		}
		catch (const std::exception& e) {
			SPDLOG_ERROR("Error during subscription callback for: {0} -> {1}", handle->name, e.what());
		}
	}
	else {
		SPDLOG_ERROR("Invalid handle or sample during subscription callback.");
	}
}

void free_subscriber_context(void* context) {
	if (context != nullptr) {
		free(context);
	}
}



//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------


int MQ_SetupZenohRawSubscription(const char* name, const char* keyexpr, ZenohSubscriptionCallBack cb) {
	if (!g_zenoh_context) {
		SPDLOG_ERROR("SetupZenohRawSubscription called, but Zenoh context is empty.");
		return -1;
	}


	subscriber_handle handle{};
	handle.name = name;
	handle.keyexpr = z_declare_keyexpr(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr));
	handle.options = z_subscriber_options_default();

	// set options here

	auto ctx = new subscriber_context();
	ctx->name = name;
	ctx->callback = cb;

	// zclosure macro does not work with c++17
	z_owned_closure_sample_t callback{};
	callback.call = handle_subscriber_callback;
	callback.context = ctx;
	callback.drop = free_subscriber_context;

	// set options here
	handle.subscriber = z_declare_subscriber(z_loan(g_zenoh_context->session), z_loan(handle.keyexpr), z_move(callback), &handle.options);

	int idx;
	{
		CriticalSection cs(&g_lock_sub);
		idx = g_subscribers.size();
		g_subscribers.push_back(std::move(handle));
	}
	return idx;
}


bool MQ_SendMessage(const char* keyexpr, uint8_t* buffer, std::size_t buffer_len, z_encoding_prefix_t encoding, bool block) {
	if (!g_zenoh_context) {
		SPDLOG_ERROR("SendMessage called, but Zenoh context is empty.");
		return false;
	}

	z_put_options_t options = z_put_options_default();
	options.encoding = z_encoding(encoding, NULL);
	options.congestion_control = block ? Z_CONGESTION_CONTROL_BLOCK : Z_CONGESTION_CONTROL_DROP;
	int res = z_put(z_loan(g_zenoh_context->session), z_keyexpr(keyexpr), buffer, buffer_len, &options);
	return res == 0;
}

// OK
void MQ_Initialize(HC_Context_Ptr& context)
{
	g_zenoh_context = context;
	InitializeCriticalSection(&g_lock_sub);
	g_event_quit    = CreateEvent(NULL, TRUE,  FALSE, NULL);
}

// OK
void MQ_Quit()
{
	SetEvent(g_event_quit);
	for (auto& handle : g_subscribers) {
		z_undeclare_subscriber(z_move(handle.subscriber));
	}
}

// OK
void MQ_Cleanup()
{

	g_subscribers.clear();
	CloseHandle(g_event_quit);
	DeleteCriticalSection(&g_lock_sub);
}
