
#include <mfapi.h>
#include "research_mode.h"
#include "server.h"
#include "utilities.h"
#include "stream_pv.h"
#include "stream_rm.h"
#include "stream_mc.h"
#include "locator.h"
#include "spatial_input.h"
#include "stream_si.h"
#include "holographic_space.h"

#include <winrt/Windows.ApplicationModel.Core.h>
#include <winrt/Windows.UI.Core.h>

using namespace winrt::Windows::UI::Core;
using namespace winrt::Windows::ApplicationModel::Core;

struct App : winrt::implements<App, IFrameworkViewSource, IFrameworkView>
{
	bool m_windowClosed = false;
	bool m_init = false;

	IFrameworkView CreateView()
	{
		return *this;
	}
	
	void Initialize(CoreApplicationView const &applicationView)
	{
		(void)applicationView;

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

		RM_Initialize();
		MC_Initialize();
		PV_Initialize();
		SI_Initialize();

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
		SI_NotifyNextFrame(HolographicSpace_GetTimestamp());
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
