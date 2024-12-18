
#include <Windows.h>
#include <atomic>
#include "extended_execution.h"
#include "log.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.ApplicationModel.h>
#include <winrt/Windows.ApplicationModel.Core.h>
#include <winrt/Windows.ApplicationModel.ExtendedExecution.Foreground.h>
#include <winrt/Windows.UI.Core.h>
#include <winrt/Windows.Storage.h>

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::ApplicationModel;
using namespace winrt::Windows::ApplicationModel::Core;
using namespace winrt::Windows::ApplicationModel::ExtendedExecution::Foreground;
using namespace winrt::Windows::UI::Core;
using namespace winrt::Windows::Storage;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static wchar_t const* g_flat_name = L"flat_mode.cfg";
static ExtendedExecutionForegroundSession g_eefs = nullptr;
static bool g_status = false;
static std::atomic<int32_t> g_interface_priority[INTERFACE_SLOTS];

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void ExtendedExecution_OnRevoked(IInspectable const& sender, ExtendedExecutionForegroundRevokedEventArgs const& args)
{
	(void)sender;
	ShowMessage("EEFS Revoked: %d", static_cast<int>(args.Reason()));
	g_status = false;
}

// OK
void ExtendedExecution_Initialize()
{
	g_eefs = ExtendedExecutionForegroundSession();
	g_eefs.Reason(ExtendedExecutionForegroundReason::Unconstrained);
	g_eefs.Description(L"Background Capture");
	g_eefs.Revoked(ExtendedExecution_OnRevoked);
	g_status = g_eefs.RequestExtensionAsync().get() == ExtendedExecutionForegroundResult::Allowed;
	ShowMessage("EEFS Result: %d", g_status);	
}

// OK
void ExtendedExecution_GetApplicationVersion(uint16_t data[4])
{
	PackageVersion version = Package::Current().Id().Version();

	data[0] = version.Major;
	data[1] = version.Minor;
	data[2] = version.Build;
	data[3] = version.Revision;
}

// OK
void ExtendedExecution_RunOnMainThread(std::function<void()> f)
{
	CoreApplication::MainView().Dispatcher().RunAsync(CoreDispatcherPriority::High, f).get();
}

// OK
void ExtendedExecution_SetFlatMode(bool flat)
{
	StorageFolder folder = ApplicationData::Current().LocalFolder();
	winrt::hstring name{ g_flat_name };

	try
	{
	if (flat)
	{
	folder.CreateFileAsync(name).get();
	}
	else
	{
	StorageFile file = folder.GetFileAsync(name).get();
	file.DeleteAsync().get();
	}
	}
	catch (...)
	{
	}
}

// OK
bool ExtendedExecution_GetFlatMode()
{
	StorageFolder folder = ApplicationData::Current().LocalFolder();
	winrt::hstring name{ g_flat_name };

	try
	{
	folder.GetFileAsync(name).get();
	return true;
	}
	catch (...)
	{
	}

	return false;
}

// OK
void ExtendedExecution_SetInterfacePriority(uint32_t id, int32_t priority)
{
	if (id >= INTERFACE_SLOTS) { return; }
	if ((priority < THREAD_PRIORITY_LOWEST) || (priority > THREAD_PRIORITY_HIGHEST)) { return; }
	g_interface_priority[id] = priority;
}

// OK
int32_t ExtendedExecution_GetInterfacePriority(uint32_t id)
{
	if (id >= INTERFACE_SLOTS) { return THREAD_PRIORITY_NORMAL; }
	return g_interface_priority[id];
}
