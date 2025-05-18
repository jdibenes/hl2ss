
function [opmode] = extended_audio_device_mixer_mode(mixer_mode, device_index, source_index, format_index)
di = bitshift(bitand(uint32(device_index + 1), uint32(0x3FF)),  2);
ds = bitshift(bitand(uint32(source_index),     uint32(0x3FF)), 12);
df = bitshift(bitand(uint32(format_index),     uint32(0x3FF)), 22);

opmode = bitor(uint32(mixer_mode), bitor(di, bitor(ds, df)));
end
