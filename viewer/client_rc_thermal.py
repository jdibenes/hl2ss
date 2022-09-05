#------------------------------------------------------------------------------
# This script suppresses default system thermal mitigations for HoloLens
# devices. Use with caution, the OS will close the server application if the
# HoloLens gets hot enough. Purpose is to maintain stable framerates by
# reducing thermal throttling at the risk of increased heat dissipation.
#------------------------------------------------------------------------------

import hl2ss

# Settings --------------------------------------------------------------------

# HoloLens address
host = '192.168.1.15'

# Suppressed devices
flags = hl2ss.PowerThermalPeripheralFlags.Cpu | hl2ss.PowerThermalPeripheralFlags.Gpu | hl2ss.PowerThermalPeripheralFlags.Dram

#------------------------------------------------------------------------------

client = hl2ss.tx_rc(host, hl2ss.IPCPort.REMOTE_CONFIGURATION)
client.suppress_platform_mitigation(flags)
