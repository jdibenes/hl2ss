
#include <mfapi.h>
#include "research_mode.h"
#include "server.h"
#include "ipc_rc.h"
#include "stream_pv.h"
#include "stream_rm.h"
#include "stream_mc.h"
#include "locator.h"
#include "spatial_input.h"
#include "stream_si.h"
#include "holographic_space.h"
#include "nfo.h"
#include "personal_video.h"
#include "spatial_mapping.h"
#include "ipc_sm.h"
#include "scene_understanding.h"
#include "ipc_su.h"
#include "voice_input.h"
#include "ipc_vi.h"
#include "stream_eet.h"
#include <algorithm>
#include <spdlog/spdlog.h>

#include <winrt/Windows.ApplicationModel.h>
#include <winrt/Windows.ApplicationModel.Core.h>
#include <winrt/Windows.UI.Core.h>

#include "hl2ss_network.h"
#include "manager.h"


using namespace winrt::Windows::ApplicationModel;
using namespace winrt::Windows::ApplicationModel::Core;
using namespace winrt::Windows::UI::Core;

struct App : winrt::implements<App, IFrameworkViewSource, IFrameworkView>
{
	bool m_windowClosed = false;
	bool m_init = false;
	HC_Context* context = NULL;

	IFrameworkView CreateView()
	{
		return *this;
	}

	void OnSuspending(IInspectable const& sender, SuspendingEventArgs const& args)
	{
		(void)sender;
		(void)args;
		spdlog::info("Application was asked to suspend - exiting.");
		if (context != nullptr) {
			spdlog::debug("Zenoh: stop services");
			StopManager(context);
			z_close(std::move(&(context->session)));
			free(context);
			context = NULL;
		}

		CoreApplication::Exit(); // Suspending is not supported
	}
	
	void Initialize(CoreApplicationView const &applicationView)
	{
		(void)applicationView;

		CoreApplication::Suspending({ this, &App::OnSuspending });

		InitializeSockets();
		MFStartup(MF_VERSION);
	}

	void Load(winrt::hstring const&)
	{
	}

	void Uninitialize()
	{
	}

	void SetWindow(CoreWindow const& window)
	{
		window.Closed({ this, &App::OnWindowClosed });

		if (m_init) { return; }

		// application defaults
		uint32_t streams_enabled = 0xFF;

		auto logger = std::make_shared<spdlog::logger>("application");
		spdlog::set_default_logger(logger);
		spdlog::set_level(spdlog::level::debug);

		spdlog::flush_every(std::chrono::milliseconds(500));
		spdlog::flush_on(spdlog::level::debug);

		SetupDebugLogSink();
		SPDLOG_INFO("Init logging");

		HolographicSpace_Initialize();
		Locator_Initialize();

		if (streams_enabled & HL2SS_ENABLE_RM) { ResearchMode_Initialize(); }
		if (streams_enabled & HL2SS_ENABLE_PV) { PersonalVideo_Initialize(); }
		if (streams_enabled & HL2SS_ENABLE_SI) { SpatialInput_Initialize(); }
		if (streams_enabled & HL2SS_ENABLE_SM) { SpatialMapping_Initialize(); }
		if (streams_enabled & HL2SS_ENABLE_SU) { SceneUnderstanding_Initialize(); }
		if (streams_enabled & HL2SS_ENABLE_VI) { VoiceInput_Initialize(); }

		// testing zenoh in uwp app
		z_owned_config_t config = z_config_default();

		context = new HC_Context(); // release
		//std::string host_name = "dev00";

		std::vector<wchar_t> buf;
		GetLocalHostname(buf);
		std::wstring host_name_w(buf.size(), '\0');
		std::copy(buf.begin(), buf.end(), host_name_w.begin());

		context->client_id = wide_string_to_string(host_name_w);
		SPDLOG_INFO("Using Hostname: {0} as client_id", context->client_id);

		context->session = z_open(z_move(config));
		if (!z_check(context->session)) {
			// error
			OutputDebugStringA("Error opening Zenoh Session");
			return;
		}

		context->streams_enabled = streams_enabled;
		context->streams_started = 0x00;
		context->valid = true;

		StartManager(context);

		m_init = true;
	}

	void Run()
	{
		auto window = CoreWindow::GetForCurrentThread();
		window.Activate();

		while (!m_windowClosed)
		{
		window.Dispatcher().ProcessEvents(CoreProcessEventsOption::ProcessAllIfPresent);

		HolographicSpace_Update();
		HolographicSpace_Clear();
		// Draw
		HolographicSpace_Present();

		}
	}

	void OnWindowClosed(CoreWindow const& sender, CoreWindowEventArgs const& args)
	{
		(void)sender;
		(void)args;

		m_windowClosed = true;
	}
};

int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR lpCmdLine, int nShowCmd)
{
	(void)hInstance;
	(void)hPrevInstance;
	(void)lpCmdLine;
	(void)nShowCmd;

	CoreApplication::Run(winrt::make<App>());
	return 0;
}
