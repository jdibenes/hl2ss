
function [mode] = extended_audio_raw_configuration(media_category, shared, audio_raw, disable_effect, enable_passthrough)
b0_2 = bitshift(bitand(uint8(media_category), uint8(7)), 0);
b3_3 = bitshift(uint8(shared), 3);
b4_4 = bitshift(uint8(0), 4);
b5_5 = bitshift(uint8(audio_raw), 5);
b6_6 = bitshift(uint8(disable_effect), 6);
b7_7 = bitshift(uint8(enable_passthrough), 7);

mode = bitor(b7_7, bitor(b6_6, bitor(b5_5, bitor(b4_4, bitor(b3_3, b0_2)))));
end
