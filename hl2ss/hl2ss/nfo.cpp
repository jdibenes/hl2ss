
#include <Windows.h>
#include "nfo.h"
#include "log.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Devices.Enumeration.h>

using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::Devices::Enumeration;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

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
    ShowMessage("Processor Architecture: %d", si.wProcessorArchitecture);
    ShowMessage("Processor Level: %d", si.wProcessorLevel);
    ShowMessage("Processor Revision: %d", si.wProcessorRevision);

    GetProcessAffinityMask(GetCurrentProcess(), &pam, &sam);

    ShowMessage("Process Affinity Mask: %llX", pam);
    ShowMessage("System Affinity Mask: %llX", sam);
}

// OK
static void PrintVariant(winrt::hstring const& name, IInspectable const& value)
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
    auto w = value.try_as<IReferenceArray<uint8_t>>();
    if (w)
    { 
    auto m = w.Value();
    text = L"{" + winrt::to_hstring(m.size()) + L", [";
    for (auto a = m.begin(); a != m.end(); a++) { text = text + winrt::hstring(a != m.begin() ? L", " : L"") + winrt::to_hstring(*a); }
    text = text + L"]}";
    };
    }

    if (type == L"Windows.Foundation.IReference`1<Boolean>")
    {
    auto w = value.try_as<bool>();
    if (w) { text = w.value() ? L"true" : L"false"; }
    }

    if (type == L"Windows.Foundation.IReference`1<UInt32>")
    {
    auto w = value.try_as<uint32_t>();
    if (w) { text = winrt::to_hstring(w.value()); }
    }

    if (type == L"Windows.Foundation.IReference`1<UInt64>")
    {
    auto w = value.try_as<uint64_t>();
    if (w) { text = winrt::to_hstring(w.value()); }
    }

    if (type == L"Windows.Foundation.IReference`1<Guid>")
    {
    auto w = value.try_as<winrt::guid>();
    if (w) { text = winrt::to_hstring(w.value()); }
    }

    if (type == L"Windows.Foundation.IReference`1<String>")
    {
    auto w = value.try_as<winrt::hstring>();
    if (w) { text = w.value(); }
    }

    if (type == L"Windows.Foundation.Collections.IMapView`2<Guid, Object>")
    {
    auto w = value.try_as<IMapView<winrt::guid, IInspectable>>();
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
void PrintProperties(IMapView<winrt::hstring, IInspectable> const& p)
{
    if (p) { for (auto const& kv : p) { PrintVariant(kv.Key(), kv.Value()); } }
}

// OK
void PrintProperties(IMapView<winrt::guid, IInspectable> const& p)
{
    if (p) { for (auto const& kv : p) { PrintVariant(winrt::to_hstring(kv.Key()), kv.Value()); } }
}

// OK
void PrintProperties(IPropertySet const& p)
{
    if (p) { for (auto const& kv : p) { PrintVariant(kv.Key(), kv.Value()); } }
}

// OK
static void PrintDeviceList(DeviceClass dc)
{
    auto list = DeviceInformation::FindAllAsync(dc).get();

    ShowMessage("Items of DeviceClass %d", static_cast<int>(dc));
    for (auto const& item : list)
    {
    ShowMessage(L"Name: %s", item.Name().c_str());
    ShowMessage(L"Id: %s", item.Id().c_str());

    ShowMessage("IsDefault: %d", static_cast<int>(item.IsDefault()));
    ShowMessage("IsEnabled: %d", static_cast<int>(item.IsEnabled()));
    ShowMessage("Kind: %d", static_cast<int>(item.Kind()));

    auto el = item.EnclosureLocation();

    if (el)
    {
    ShowMessage("InDock: %d", static_cast<int>(el.InDock()));
    ShowMessage("InLid: %d", static_cast<int>(el.InLid()));
    ShowMessage("Panel: %d", static_cast<int>(el.Panel()));
    ShowMessage("RotationAngleInDegreesClockwise: %d", static_cast<int>(el.RotationAngleInDegreesClockwise()));
    }
        
    auto pa = item.Pairing();

    if (pa)
    {
    ShowMessage("CanPair: %d", static_cast<int>(pa.CanPair()));
    ShowMessage("IsPaired: %d", static_cast<int>(pa.IsPaired()));
    ShowMessage("ProtectionLevel: %d", static_cast<int>(pa.ProtectionLevel()));
    }

    ShowMessage("Properties:");
    PrintProperties(item.Properties());
    }
}

// OK
void PrintVideoCaptureList()
{
    PrintDeviceList(DeviceClass::VideoCapture);
}

// OK
void PrintAudioCaptureList()
{
    PrintDeviceList(DeviceClass::AudioCapture);
}

// OK
static winrt::hstring GetBuiltInId(DeviceClass dc)
{
    winrt::hstring const key = L"System.Devices.ContainerId";

    auto list = DeviceInformation::FindAllAsync(dc).get();
    for (auto const& item : list)
    {
    auto p = item.Properties();
    if (!p) { continue; }
    if (!p.HasKey(key)) { continue; }
    auto v = p.Lookup(key);
    auto g = v.try_as<winrt::guid>();
    if (!g) { continue; }
    auto d = g.value();
    if ((d.Data1 != 0) || (d.Data2 != 0) || (d.Data3 != 0) || ((*reinterpret_cast<uint64_t*>(d.Data4)) != 0xFFFFFFFFFFFFFFFFULL)) { continue; }
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
static void GetDeviceIds(DeviceClass dc, std::vector<winrt::hstring>& ids)
{
    auto list = DeviceInformation::FindAllAsync(dc).get();
    for (auto const& item : list)
    {
    ids.push_back(item.Id());
    }
}

// OK
static void GetDeviceIdsAndNames(DeviceClass dc, std::vector<winrt::hstring>& ids, std::vector<winrt::hstring>& names)
{
    auto list = DeviceInformation::FindAllAsync(dc).get();
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
