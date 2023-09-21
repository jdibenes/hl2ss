
classdef h26x_encoder_property
properties (Constant)
    CODECAPI_AVEncCommonRateControlMode     = uint64( 0);
    CODECAPI_AVEncCommonQuality             = uint64( 1);
    CODECAPI_AVEncAdaptiveMode              = uint64( 2);
    CODECAPI_AVEncCommonBufferSize          = uint64( 3);
    CODECAPI_AVEncCommonMaxBitRate          = uint64( 4);
    CODECAPI_AVEncCommonMeanBitRate         = uint64( 5);
    CODECAPI_AVEncCommonQualityVsSpeed      = uint64( 6);
    CODECAPI_AVEncH264CABACEnable           = uint64( 7);
    CODECAPI_AVEncH264SPSID                 = uint64( 8);
    CODECAPI_AVEncMPVDefaultBPictureCount   = uint64( 9);
    CODECAPI_AVEncMPVGOPSize                = uint64(10);
    CODECAPI_AVEncNumWorkerThreads          = uint64(11);
    CODECAPI_AVEncVideoContentType          = uint64(12);
    CODECAPI_AVEncVideoEncodeQP             = uint64(13);
    CODECAPI_AVEncVideoForceKeyFrame        = uint64(14);
    CODECAPI_AVEncVideoMinQP                = uint64(15);
    CODECAPI_AVLowLatencyMode               = uint64(16);
    CODECAPI_AVEncVideoMaxQP                = uint64(17);
    CODECAPI_VideoEncoderDisplayContentType = uint64(18);
end
end
