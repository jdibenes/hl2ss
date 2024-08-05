
function [opmode] = extended_audio_device_mixer_mode(mixer_mode, device)
DEVICE_BASE = uint32(0x00000004);
opmode = uint32(mixer_mode) + (DEVICE_BASE * uint32(device + 1));
end
