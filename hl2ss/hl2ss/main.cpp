
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

#include <winrt/Windows.ApplicationModel.h>
#include <winrt/Windows.ApplicationModel.Core.h>
#include <winrt/Windows.UI.Core.h>

#include "zenoh.h"


using namespace winrt::Windows::ApplicationModel;
using namespace winrt::Windows::ApplicationModel::Core;
using namespace winrt::Windows::UI::Core;

struct App : winrt::implements<App, IFrameworkViewSource, IFrameworkView>
{
	bool m_windowClosed = false;
	bool m_init = false;

	IFrameworkView CreateView()
	{
		return *this;
	}

	void OnSuspending(IInspectable const& sender, SuspendingEventArgs const& args)
	{
		(void)sender;
		(void)args;

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

		HolographicSpace_Initialize();
		Locator_Initialize();
		ResearchMode_Initialize();
		SpatialInput_Initialize();
		PersonalVideo_Initialize();
		SpatialMapping_Initialize();
		SceneUnderstanding_Initialize();
		VoiceInput_Initialize();

		// testing zenoh in uwp app
		z_owned_config_t config = z_config_default();

		z_owned_session_t zs = z_open(z_move(config));
		if (!z_check(zs)) {
			// error message
		}

		// some global parameters that can be modified when used as plugin...
		const char* client_id = "dev00";
		uint8_t eye_fps = 60;


		RM_Initialize(client_id, z_loan(zs));
		MC_Initialize(client_id, z_loan(zs));

		H26xFormat format{};
		format.width = 640;
		format.height = 480;
		format.framerate = 30;
		PV_Initialize(client_id, z_loan(zs), true, format);

		SI_Initialize(client_id, z_loan(zs));
		RC_Initialize(client_id, z_loan(zs));
		SM_Initialize(client_id, z_loan(zs));
		SU_Initialize(client_id, z_loan(zs));
		VI_Initialize(client_id, z_loan(zs));

		EET_Initialize(client_id, z_loan(zs), eye_fps);

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
