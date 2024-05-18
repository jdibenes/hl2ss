
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
#include "stream_ea.h"
#include "extended_execution.h"
#include "stream_ev.h"

#include <winrt/Windows.ApplicationModel.h>
#include <winrt/Windows.ApplicationModel.Core.h>
#include <winrt/Windows.UI.Core.h>

using namespace winrt::Windows::ApplicationModel;
using namespace winrt::Windows::ApplicationModel::Core;
using namespace winrt::Windows::UI::Core;

struct App : winrt::implements<App, IFrameworkViewSource, IFrameworkView>
{
	bool m_windowClosed = false;
	bool m_init = false;
	bool m_flat = false;

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

		m_flat = ExtendedExecution_GetFlatMode();
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

		if (!m_flat)
		{
		HolographicSpace_Initialize();
		}

		Locator_Initialize();
		ResearchMode_Initialize();
		SpatialInput_Initialize();
		PersonalVideo_Initialize();
		SpatialMapping_Initialize();
		SceneUnderstanding_Initialize();
		VoiceInput_Initialize();

		RM_Initialize();
		MC_Initialize();
		PV_Initialize();
		SI_Initialize();
		RC_Initialize();
		SM_Initialize();
		SU_Initialize();
		VI_Initialize();
		EET_Initialize();
		EA_Initialize();
		EV_Initialize();

		ExtendedExecution_Request();
		
		m_init = true;
	}

	void Run()
	{
		auto window = CoreWindow::GetForCurrentThread();
		window.Activate();

		while (!m_windowClosed)
		{
		window.Dispatcher().ProcessEvents(CoreProcessEventsOption::ProcessAllIfPresent);

		if (!m_flat)
		{
		HolographicSpace_Update();
		HolographicSpace_Clear();
		// Draw
		HolographicSpace_Present();
		}
		}

		CoreApplication::Exit();
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
