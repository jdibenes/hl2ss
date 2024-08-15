
classdef sink_pv < matlab.System
    properties
    
    end

    properties (Nontunable)
        host           = '192.168.1.7'
        port           = hl2ss.stream_port.PERSONAL_VIDEO
        width          = 1920
        height         = 1080
        framerate      = 30
        chunk          = 4096
        mode           = hl2ss.stream_mode.MODE_1
        divisor        = 1
        profile        = hl2ss.video_profile.H265_MAIN
        level          = hl2ss.h26x_level.DEFAULT
        bitrate        = 0
        options        = [hl2ss.h26x_encoder_property.CODECAPI_AVEncMPVGOPSize, 30]
        decoded_format = hl2ss.pv_decoded_format.RGB
        buffer_size    = 30 * 10
        sample_time    = 1 / 30

        enable_mrc                 = false
        hologram_composition       = true
        recording_indicator        = false
        video_stabilization        = false
        blank_protected            = false
        show_mesh                  = false
        shared                     = false
        global_opacity             = 0.9
        output_width               = 0.0
        output_height              = 0.0
        video_stabilization_length = 0
        hologram_perspective       = hl2ss.hologram_perspective.PV

        time_preference = hl2ss.grab_preference.PREFER_NEAREST
        tiebreak_right  = false
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        client
        definition_pv
    end

    methods (Access = protected)
        function [image_size] = getImageSize(obj)
            if     (obj.decoded_format == hl2ss.pv_decoded_format.BGR)
                image_size = [obj.height, obj.width, 3];
            elseif (obj.decoded_format == hl2ss.pv_decoded_format.RGB)
                image_size = [obj.height, obj.width, 3];
            elseif (obj.decoded_format == hl2ss.pv_decoded_format.BGRA)
                image_size = [obj.height, obj.width, 4];
            elseif (obj.decoded_format == hl2ss.pv_decoded_format.RGBA)
                image_size = [obj.height, obj.width, 4];
            elseif (obj.decoded_format == hl2ss.pv_decoded_format.GRAY)
                image_size = [obj.height, obj.width];
            end
        end

        function setupImpl(obj)
            obj.definition_pv = ...
                struct('frame_index', zeros([1, 1],             'int64' ), ...
                       'status',      zeros([1, 1],             'int32' ), ...
                       'timestamp',   zeros([1, 1],             'uint64'), ...
                       'image',       zeros(obj.getImageSize(), 'uint8' ), ...
                       'intrinsics',  zeros([4, 1],             'single'), ...
                       'exposure',    zeros([3, 1],             'uint64'), ...
                       'imaging',     zeros([4, 1],             'uint32'), ...
                       'gains',       zeros([5, 1],             'single'), ...
                       'pose',        zeros([4, 4],             'single'));

            coder.extrinsic('hl2ss_matlab')

            obj.client = hl2ss.mt.sink_pv(obj.host, obj.port, obj.width, obj.height, obj.framerate, obj.chunk, obj.mode, obj.divisor, obj.profile, obj.level, obj.bitrate, obj.options, obj.decoded_format, obj.buffer_size, @hl2ss_matlab);
            
            obj.client.start_subsystem(obj.enable_mrc, obj.hologram_composition, obj.recording_indicator, obj.video_stabilization, obj.blank_protected, obj.show_mesh, obj.shared, obj.global_opacity, obj.output_width, obj.output_height, obj.video_stabilization_length, obj.hologram_perspective)
            obj.client.open()
        end

        function [frame_index, status, timestamp, image, intrinsics, pose, exposure, imaging, gains] = stepImpl(obj, sync, index)
            if (sync <= 0)
                response = obj.client.get_packet_by_index(index);
            else
                response = obj.client.get_packet_by_timestamp(sync, obj.time_preference, obj.tiebreak_right);
            end

            coder.extrinsic('hl2ss.ms.unpack_pv')

            data = obj.definition_pv;
            data = hl2ss.ms.unpack_pv(response, obj.mode, obj.getImageSize());

            frame_index = data.frame_index;
            status      = data.status;
            timestamp   = data.timestamp;
            image       = data.image;
            intrinsics  = data.intrinsics;
            exposure    = data.exposure;
            imaging     = data.imaging;
            gains       = data.gains;
            pose        = data.pose;
        end

        function resetImpl(obj)

        end

        function releaseImpl(obj)
            obj.client.close();
            obj.client.stop_subsystem()
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8, out9] = getOutputSizeImpl(obj)
            out1 = [1, 1];
            out2 = [1, 1];
            out3 = [1, 1];
            out4 = obj.getImageSize();
            out5 = [4, 1];
            out6 = [4, 4];
            out7 = [3, 1];
            out8 = [4, 1];
            out9 = [5, 1];
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8, out9] = getOutputDataTypeImpl(obj)
            out1 = 'int64';
            out2 = 'int32';
            out3 = 'uint64';
            out4 = 'uint8';
            out5 = 'single';
            out6 = 'single';
            out7 = 'uint64';
            out8 = 'uint32';
            out9 = 'single';
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8, out9] = isOutputComplexImpl(obj)
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
            out7 = false;
            out8 = false;
            out9 = false;
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8, out9] = isOutputFixedSizeImpl(obj)
            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
            out6 = true;
            out7 = true;
            out8 = true;
            out9 = true;
        end

        function sts = getSampleTimeImpl(obj)
            sts = obj.createSampleTime("Type", "Discrete", "SampleTime", obj.sample_time);
        end
    end
end
