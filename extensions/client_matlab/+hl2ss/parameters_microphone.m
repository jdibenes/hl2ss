
classdef parameters_microphone
properties (Constant)
    ARRAY_CHANNELS     = 5
    ARRAY_TOP_LEFT     = 1
    ARRAY_TOP_CENTER   = 2
    ARRAY_TOP_RIGHT    = 3
    ARRAY_BOTTOM_LEFT  = 4
    ARRAY_BOTTOM_RIGHT = 5

    SAMPLE_RATE    = 48000;
    CHANNELS       = 2;
    GROUP_SIZE_RAW = 768;
    GROUP_SIZE_AAC = 1024;
end
end
