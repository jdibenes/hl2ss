
#include <Windows.h>
#include <combaseapi.h>
#include "nfo.h"
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
    for (auto const& hostname : NetworkInformation::GetHostNames())
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
static void PrintVariant(winrt::hstring const& name, winrt::Windows::Foundation::IInspectable const& value)
{
    if (!value)
    {
        ShowMessage(L"%s : ? -> [NULL]", name.c_str());
        return;
    }

    winrt::hstring type = winrt::get_class_name(value);
    winrt::hstring text = L"[?]";

    if (type == L"Windows.Foundation.IReferenceArray`1<UInt8>") // blob
    {
        auto const& w = value.try_as<winrt::Windows::Foundation::IReferenceArray<uint8_t>>();
        if (w)
        { 
            auto const& m = w.Value();
            text = L"{" + winrt::to_hstring(m.size()) + L", [";
            for (auto a = m.begin(); a != m.end(); a++) { text = text + winrt::hstring(a != m.begin() ? L", " : L"") + winrt::to_hstring(*a); }
            text = text + L"]}";
        };
    }
    if (type == L"Windows.Foundation.IReference`1<Boolean>")
    {
        auto const& w = value.try_as<bool>();
        if (w) { text = w.value() ? L"true" : L"false"; }
    }
    if (type == L"Windows.Foundation.IReference`1<UInt32>")
    {
        auto const& w = value.try_as<uint32_t>();
        if (w) { text = winrt::to_hstring(w.value()); }
    }
    if (type == L"Windows.Foundation.IReference`1<UInt64>")
    {
        auto const& w = value.try_as<uint64_t>();
        if (w) { text = winrt::to_hstring(w.value()); }
    }
    if (type == L"Windows.Foundation.IReference`1<Guid>")
    {
        auto const& w = value.try_as<winrt::guid>();
        if (w) { text = winrt::to_hstring(w.value()); }
    }
    if (type == L"Windows.Foundation.IReference`1<String>")
    {
        auto const& w = value.try_as<winrt::hstring>();
        if (w) { text = w.value(); }
    }
    if (type == L"Windows.Foundation.Collections.IMapView`2<Guid, Object>")
    {
        auto const& w = value.try_as<winrt::Windows::Foundation::Collections::IMapView<winrt::guid, winrt::Windows::Foundation::IInspectable>>();
        if (w)
        {
            ShowMessage(L"SET %s BEGIN", name.c_str());
            PrintProperties(w);
            ShowMessage(L"SET %s END", name.c_str());
        }
    }

    ShowMessage(L"%s : %s -> %s", name.c_str(), type.c_str(), text.c_str());
}

// OK
void PrintProperties(winrt::Windows::Foundation::Collections::IMapView<winrt::hstring, winrt::Windows::Foundation::IInspectable> const& p)
{
    if (p) { for (auto const& kv : p) { PrintVariant(kv.Key(), kv.Value()); } }
}

// OK
void PrintProperties(winrt::Windows::Foundation::Collections::IMapView<winrt::guid, winrt::Windows::Foundation::IInspectable> const& p)
{
    if (p) { for (auto const& kv : p) { PrintVariant(winrt::to_hstring(kv.Key()), kv.Value()); } }
}

// OK
void PrintProperties(winrt::Windows::Foundation::Collections::IPropertySet const& p)
{
    if (p) { for (auto const& kv : p) { PrintVariant(kv.Key(), kv.Value()); } }
}

// OK
void PrintDeviceList(DeviceClass dc)
{
    auto const& list = DeviceInformation::FindAllAsync(dc).get();

    ShowMessage("Items of DeviceClass %d", (int)dc);
    for (auto const& item : list)
    {
    ShowMessage(L"Name: %s", item.Name().c_str());
    ShowMessage(L"Id: %s", item.Id().c_str());

    ShowMessage("IsDefault: %d", (int)item.IsDefault());
    ShowMessage("IsEnabled: %d", (int)item.IsEnabled());
    ShowMessage("Kind: %d", (int)item.Kind());

    auto const& el = item.EnclosureLocation();

    if (el)
    {
    ShowMessage("InDock: %d", (int)el.InDock());
    ShowMessage("InLid: %d", (int)el.InLid());
    ShowMessage("Panel: %d", (int)el.Panel());
    ShowMessage("RotationAngleInDegreesClockwise: %d", (int)el.RotationAngleInDegreesClockwise());
    }
        
    auto const& pa = item.Pairing();

    if (pa)
    {
    ShowMessage("CanPair: %d", (int)pa.CanPair());
    ShowMessage("IsPaired: %d", (int)pa.IsPaired());
    ShowMessage("ProtectionLevel: %d", (int)pa.ProtectionLevel());
    }

    ShowMessage("Properties:");
    PrintProperties(item.Properties());
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

    auto const& list = DeviceInformation::FindAllAsync(dc).get();
    for (auto const& item : list)
    {
    auto const& p = item.Properties();
    if (!p) { continue; }
    if (!p.HasKey(key)) { continue; }
    auto const& v = p.Lookup(key);
    auto const& g = v.try_as<winrt::guid>();
    if (!g) { continue; }
    auto const& d = g.value();
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
    auto const& list = DeviceInformation::FindAllAsync(dc).get();
    for (auto const& item : list)
    {
    ids.push_back(item.Id());
    }
}

// OK
void GetDeviceIdsAndNames(DeviceClass dc, std::vector<winrt::hstring>& ids, std::vector<winrt::hstring>& names)
{
    auto const& list = DeviceInformation::FindAllAsync(dc).get();
    for (auto const& item : list)
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
