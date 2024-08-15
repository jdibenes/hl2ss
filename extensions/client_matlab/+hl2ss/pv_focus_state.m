
classdef pv_focus_state
properties (Constant)
    UNINITIALIZED = uint32(0);
    LOST          = uint32(1);
    SEARCHING     = uint32(2);
    FOCUSED       = uint32(3);
    FAILED        = uint32(4);
end
end
