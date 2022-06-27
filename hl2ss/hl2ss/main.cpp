
#include <mfapi.h>

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Networking.Sockets.h>
#include <winrt/Windows.ApplicationModel.Activation.h>
#include <winrt/Windows.ApplicationModel.Core.h>
#include <winrt/Windows.UI.Core.h>
#include <iostream>
#include <winsock2.h>

//#include "Cannon/MixedReality.h"

#include "research_mode.h"
#include "server.h"
#include "utilities.h"
#include "stream_pv.h"
#include "stream_rm.h"
#include "stream_mc.h"
//#include "stream_mr.h"
#include "locator.h"

struct App : winrt::implements<App, winrt::Windows::ApplicationModel::Core::IFrameworkViewSource, winrt::Windows::ApplicationModel::Core::IFrameworkView>
{
	bool windowClosed = false;
	//MixedReality m_mixedReality;
	bool mixedInit = false;

	winrt::Windows::ApplicationModel::Core::IFrameworkView CreateView()
	{
		return *this;
	}
	
	void Initialize(winrt::Windows::ApplicationModel::Core::CoreApplicationView const &applicationView)
	{
		//DrawCall::Initialize();

		
		

		InitializeSockets();
		MFStartup(MF_VERSION);
		ResearchMode_Initialize();


		Locator_Initialize();

		winrt::Windows::Perception::Spatial::SpatialCoordinateSystem world = Locator_GetWorldCoordinateSystem();

		PV_SetWorldFrame(world);
		RM_SetWorldCoordinateSystem(world);

		RM_Initialize();
		MC_Initialize();
		PV_Initialize();
		
		
	}

	void Load(winrt::hstring const&)
	{
	}

	void Uninitialize()
	{
	}

	void SetWindow(winrt::Windows::UI::Core::CoreWindow const& window)
	{
		window.Closed({ this, &App::OnWindowClosed });
		ShowMessage("SetWindowCalled");

		if (mixedInit) { return; }

		//m_mixedReality.EnableMixedReality();
		//m_mixedReality.EnableSurfaceMapping();
//g_mixedreality.EnableQRCodeTracking();
		//m_mixedReality.EnableEyeTracking();

		

		//MR_Initialize();
		//m_mixedReality = MixedReality();
		
		mixedInit = true;
		
	}

	void Run()
	{
		auto window = winrt::Windows::UI::Core::CoreWindow::GetForCurrentThread();
		window.Activate();

		while (!windowClosed)
		{
			winrt::Windows::UI::Core::CoreWindow::GetForCurrentThread().Dispatcher().ProcessEvents(winrt::Windows::UI::Core::CoreProcessEventsOption::ProcessAllIfPresent);
			//m_mixedReality.Update();
			//MR_Update();
			//m_mixedReality->Update();
			//m_main->Update();
			//m_main->Render();
			Sleep(16);
		}
	}

	void OnWindowClosed(winrt::Windows::UI::Core::CoreWindow const& sender, winrt::Windows::UI::Core::CoreWindowEventArgs const& args)
	{
		windowClosed = true;
	}
};


int WINAPI wWinMain(HINSTANCE, HINSTANCE, PWSTR, int)
{
	winrt::Windows::ApplicationModel::Core::CoreApplication::Run(winrt::make<App>());
	return 0;
}
