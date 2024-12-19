
#include <Windows.h>
#include "ipl.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.ApplicationModel.h>
#include <winrt/Windows.ApplicationModel.Core.h>
#include <winrt/Windows.UI.Core.h>

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::ApplicationModel;
using namespace winrt::Windows::ApplicationModel::Core;
using namespace winrt::Windows::UI::Core;

struct App : winrt::implements<App, IFrameworkViewSource, IFrameworkView>
{
	bool m_windowClosed = false;

	IFrameworkView CreateView();
	void OnWindowClosed(CoreWindow const& sender, CoreWindowEventArgs const& args);
	void OnSuspending(IInspectable const& sender, SuspendingEventArgs const& args);
	void Initialize(CoreApplicationView const& applicationView);
	void Load(winrt::hstring const& entryPoint);
	void SetWindow(CoreWindow const& window);
	void Run();
	void Uninitialize();
};

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
IFrameworkView App::CreateView()
{
	return *this;
}

// OK
void App::OnWindowClosed(CoreWindow const& sender, CoreWindowEventArgs const& args)
{
	(void)sender;
	(void)args;

	m_windowClosed = true;
}

// OK
void App::OnSuspending(IInspectable const& sender, SuspendingEventArgs const& args)
{
	(void)sender;
	(void)args;

	CoreApplication::Exit(); // Suspending is not supported
}

// OK
void App::Initialize(CoreApplicationView const& applicationView)
{
	(void)applicationView;
	CoreApplication::Suspending({ this, &App::OnSuspending });
}

// OK
void App::Load(winrt::hstring const& entryPoint)
{
	(void)entryPoint;
	HL2SS_Load(true);
}

// OK
void App::SetWindow(CoreWindow const& window)
{
	window.Closed({ this, &App::OnWindowClosed });
}

// OK
void App::Run()
{
	auto window = CoreWindow::GetForCurrentThread();
	window.Activate();

	while (!m_windowClosed)
	{
	window.Dispatcher().ProcessEvents(CoreProcessEventsOption::ProcessAllIfPresent);

	HL2SS_Process_HS();
	HL2SS_Process_MQ();
	HL2SS_Process_MQX();
	HL2SS_Process_EE();
	}
}

// OK
void App::Uninitialize()
{
}

//*****************************************************************************
// Entry Point
//*****************************************************************************

// OK
int WINAPI wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, PWSTR lpCmdLine, int nShowCmd)
{
	(void)hInstance;
	(void)hPrevInstance;
	(void)lpCmdLine;
	(void)nShowCmd;

	CoreApplication::Run(winrt::make<App>());
	return 0;
}
