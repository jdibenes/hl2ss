
#include <Microsoft.MixedReality.PowerThermalNotification.h>
#include "utilities.h"

#define PTPF_MASK 0x7F

using namespace Microsoft::MixedReality::PowerThermalNotification;

void ThermalControl_SuppressPlatformMitigation(uint8_t flags)
{
    auto ptn = PowerThermalNotification::GetForCurrentProcess();
    ptn->SetSuppressedPlatformMitigationForPeripherals((PowerThermalPeripheralFlags)(flags & PTPF_MASK));
    ShowMessage("ThermalControl: Suppression flags set to %d", flags);
}
