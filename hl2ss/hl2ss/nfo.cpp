
#include <Windows.h>
#include <combaseapi.h>
#include "log.h"

#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.ApplicationModel.h>
#include <winrt/Windows.Networking.h>
#include <winrt/Windows.Networking.Connectivity.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Devices.Enumeration.h>

using namespace winrt::Windows::ApplicationModel;
using namespace winrt::Windows::Networking;
using namespace winrt::Windows::Networking::Connectivity;
using namespace winrt::Windows::Devices::Enumeration;

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

// OK
void PrintDeviceList(DeviceClass dc)
{
    auto list = DeviceInformation::FindAllAsync(dc).get();

    ShowMessage("Items of DeviceClass %d", (int)dc);
    for (auto item : list)
    {
    ShowMessage(L"Name: %s", item.Name().c_str());
    ShowMessage(L"Id: %s", item.Id().c_str());

    ShowMessage("IsDefault: %d", (int)item.IsDefault());
    ShowMessage("IsEnabled: %d", (int)item.IsEnabled());
    ShowMessage("Kind: %d", (int)item.Kind());

    auto el = item.EnclosureLocation();

    if (el)
    {
    ShowMessage("InDock: %d", (int)el.InDock());
    ShowMessage("InLid: %d", (int)el.InLid());
    ShowMessage("Panel: %d", (int)el.Panel());
    ShowMessage("RotationAngleInDegreesClockwise: %d", (int)el.RotationAngleInDegreesClockwise());
    }
        
    auto pa = item.Pairing();

    if (pa)
    {
    ShowMessage("CanPair: %d", (int)pa.CanPair());
    ShowMessage("IsPaired: %d", (int)pa.IsPaired());
    ShowMessage("ProtectionLevel: %d", (int)pa.ProtectionLevel());
    }

    ShowMessage("Properties:");
    auto p = item.Properties();
    if (p)
    {
    for (auto y : p)
    {
    auto s = y.Value().try_as<winrt::hstring>();
    if (s)
    {
    ShowMessage(L"%s: %s", y.Key().c_str(), s.value().c_str());
    continue;
    }
    auto i = y.Value().try_as<int>();
    if (i)
    {
    ShowMessage(L"%s: %d", y.Key().c_str(), i.value());
    continue;
    }
    auto b = y.Value().try_as<bool>();
    if (b)
    {
    ShowMessage(L"%s: %d", y.Key().c_str(), (int)b.value());
    continue;
    }
    auto g = y.Value().try_as<winrt::guid>();
    if (g)
    {
    wchar_t buffer[39];
    memset(buffer, 0, sizeof(buffer));
    StringFromGUID2(g.value(), buffer, sizeof(buffer) / sizeof(wchar_t));
    buffer[38] = L'\0';
    ShowMessage(L"%s: %s", y.Key().c_str(), buffer);
    continue;
    }
    ShowMessage(L"%s: %s", y.Key().c_str(), L"[?]");
    }
    }
    }
}

// OK
void PrintAudioCaptureList()
{
    PrintDeviceList(DeviceClass::AudioCapture);
}

// OK
void PrintVideoCaptureList()
{
    PrintDeviceList(DeviceClass::VideoCapture);
}

// OK
winrt::hstring GetBuiltInId(DeviceClass dc)
{
    winrt::hstring const key = L"System.Devices.ContainerId";

    auto list = DeviceInformation::FindAllAsync(dc).get();
    for (auto item : list)
    {
    auto p = item.Properties();
    if (!p) { continue; }
    if (!p.HasKey(key)) { continue; }
    auto v = p.Lookup(key);
    auto g = v.try_as<winrt::guid>();
    if (!g) { continue; }
    auto d = g.value();
    if ((d.Data1 != 0) || (d.Data2 != 0) || (d.Data3 != 0) || ((*(uint64_t*)d.Data4) != 0xFFFFFFFFFFFFFFFFULL)) { continue; }
    return item.Id();
    }

    return list.GetAt(0).Id();
}

// OK
winrt::hstring GetBuiltInVideoCaptureId()
{
    return GetBuiltInId(DeviceClass::VideoCapture);
}

// OK
winrt::hstring GetBuiltInAudioCaptureId()
{
    return GetBuiltInId(DeviceClass::AudioCapture);
}

// OK
void GetDeviceIds(DeviceClass dc, std::vector<winrt::hstring>& ids)
{
    auto list = DeviceInformation::FindAllAsync(dc).get();
    for (auto item : list)
    {
    ids.push_back(item.Id());
    }
}

// OK
void GetDeviceIdsAndNames(DeviceClass dc, std::vector<winrt::hstring>& ids, std::vector<winrt::hstring>& names)
{
    auto list = DeviceInformation::FindAllAsync(dc).get();
    for (auto item : list)
    {
    ids.push_back(item.Id());
    names.push_back(item.Name());
    }
}

// OK
void GetVideoCaptureIds(std::vector<winrt::hstring>& ids)
{
    GetDeviceIds(DeviceClass::VideoCapture, ids);
}

// OK
void GetAudioCaptureIds(std::vector<winrt::hstring>& ids)
{
    GetDeviceIds(DeviceClass::AudioCapture, ids);
}

// OK
void GetAudioCaptureIdsAndNames(std::vector<winrt::hstring>& ids, std::vector<winrt::hstring>& names)
{
    GetDeviceIdsAndNames(DeviceClass::AudioCapture, ids, names);
}
