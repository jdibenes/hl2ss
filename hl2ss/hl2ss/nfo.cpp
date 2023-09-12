
#include <Windows.h>
#include "log.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.ApplicationModel.h>
#include <winrt/Windows.Networking.h>
#include <winrt/Windows.Networking.Connectivity.h>

using namespace winrt::Windows::ApplicationModel;
using namespace winrt::Windows::Networking;
using namespace winrt::Windows::Networking::Connectivity;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

// OK
void GetApplicationVersion(uint16_t data[4])
{
    PackageVersion version = Package::Current().Id().Version();
    data[0] = version.Major;
    data[1] = version.Minor;
    data[2] = version.Build;
    data[3] = version.Revision;
}

// OK
void GetLocalIPv4Address(std::vector<wchar_t> &address)
{
    for (auto hostname : NetworkInformation::GetHostNames())
    {
    if (hostname.Type() == HostNameType::Ipv4)
    {
    wchar_t const* str = hostname.ToString().c_str();
    address.resize(wcslen(str) + 1);
    wcscpy_s(address.data(), address.size(), str);
    return;
    }
    }
}

// OK
void PrintSystemInfo()
{
    SYSTEM_INFO si;
    DWORD_PTR pam;
    DWORD_PTR sam;

    GetSystemInfo(&si);

    ShowMessage("Active Processor Mask: %llX", si.dwActiveProcessorMask);
    ShowMessage("Allocation Granularity: %d", si.dwAllocationGranularity);
    ShowMessage("Number of Processors: %d", si.dwNumberOfProcessors);
    ShowMessage("Page Size: %d", si.dwPageSize);
    ShowMessage("Maximum Application Address: %p", si.lpMaximumApplicationAddress);
    ShowMessage("Minimum Application Address: %p", si.lpMinimumApplicationAddress);
    ShowMessage("Processor Architecture: %d", (DWORD)si.wProcessorArchitecture);
    ShowMessage("Processor Level: %d", (DWORD)si.wProcessorLevel);
    ShowMessage("Processor Revision: %d", (DWORD)si.wProcessorRevision);

    GetProcessAffinityMask(GetCurrentProcess(), &pam, &sam);

    ShowMessage("Process Affinity Mask: %llX", pam);
    ShowMessage("System Affinity Mask: %llX", sam);
}
