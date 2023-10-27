
#include "log.h"

#include <winrt/Windows.Foundation.h>

#include <winrt/Windows.ApplicationModel.h>
#include <winrt/Windows.ApplicationModel.ExtendedExecution.h>
#include <winrt/Windows.ApplicationModel.ExtendedExecution.Foreground.h>

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::ApplicationModel;
using namespace winrt::Windows::ApplicationModel::ExtendedExecution;
using namespace winrt::Windows::ApplicationModel::ExtendedExecution::Foreground;

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

static ExtendedExecutionForegroundSession g_eefs = nullptr;
static bool g_status = false;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
static void ExtendedExecution_OnRevoked(IInspectable const& sender, ExtendedExecutionForegroundRevokedEventArgs const& args)
{
	(void)sender;
	ShowMessage("EEFS Revoked: %d", (int)args.Reason());
	g_status = false;
}

// OK
void ExtendedExecution_Request()
{
	g_eefs = ExtendedExecutionForegroundSession();
	g_eefs.Reason(ExtendedExecutionForegroundReason::Unconstrained);
	g_eefs.Description(L"Background Capture");
	g_eefs.Revoked(ExtendedExecution_OnRevoked);

	ExtendedExecutionForegroundResult result = g_eefs.RequestExtensionAsync().get();
	if (result == ExtendedExecutionForegroundResult::Allowed) { g_status = true; }
	ShowMessage("EEFS Result: %d", (int)result);	
}

// OK
bool ExtendedExecution_Status()
{
	return g_status;
}
