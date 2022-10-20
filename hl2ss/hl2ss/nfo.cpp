
#include <winrt/Windows.ApplicationModel.h>

using namespace winrt::Windows::ApplicationModel;

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

void GetApplicationVersion(uint16_t data[4])
{
    PackageVersion version = Package::Current().Id().Version();
    data[0] = version.Major;
    data[1] = version.Minor;
    data[2] = version.Build;
    data[3] = version.Revision;
}
