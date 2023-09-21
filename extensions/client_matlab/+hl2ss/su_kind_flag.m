
classdef su_kind_flag
properties (Constant)
    Background         = uint8(  1);
    Wall               = uint8(  2);
    Floor              = uint8(  4);
    Ceiling            = uint8(  8);
    Platform           = uint8( 16);
    Unknown            = uint8( 32);
    World              = uint8( 64);
    CompletelyInferred = uint8(128);
end
end
