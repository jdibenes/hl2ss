
classdef parameters_microphone
properties (Constant)
    ARRAY_CHANNELS     = 5
    ARRAY_TOP_LEFT     = 0
    ARRAY_TOP_CENTER   = 1
    ARRAY_TOP_RIGHT    = 2
    ARRAY_BOTTOM_LEFT  = 3
    ARRAY_BOTTOM_RIGHT = 4

    SAMPLE_RATE    = 48000;
    CHANNELS       = 2;
    GROUP_SIZE_RAW = 768;
    GROUP_SIZE_AAC = 1024;
end
end
