
#pragma once

#include <vector>
#include <stdint.h>

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>

void PrintSystemInfo();
void PrintProperties(winrt::Windows::Foundation::Collections::IMapView<winrt::hstring, winrt::Windows::Foundation::IInspectable> const& p);
void PrintProperties(winrt::Windows::Foundation::Collections::IMapView<winrt::guid, winrt::Windows::Foundation::IInspectable> const& p);
void PrintProperties(winrt::Windows::Foundation::Collections::IPropertySet const& p);
void PrintVideoCaptureList();
void PrintAudioCaptureList();
winrt::hstring GetBuiltInVideoCaptureId();
winrt::hstring GetBuiltInAudioCaptureId();
void GetVideoCaptureIds(std::vector<winrt::hstring>& ids);
void GetAudioCaptureIds(std::vector<winrt::hstring>& ids);
void GetAudioCaptureIdsAndNames(std::vector<winrt::hstring>& ids, std::vector<winrt::hstring>& names);
