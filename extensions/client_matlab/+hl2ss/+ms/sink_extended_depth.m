
classdef sink_extended_depth < matlab.System
    properties
    
    end

    properties (Nontunable)
        host           = '192.168.1.7'
        port           = hl2ss.stream_port.EXTENDED_DEPTH
        chunk          = 4096
        width          = 640
        height         = 360
        media_index    = 0xFFFFFFFF
        stride_mask    = 63
        mode           = 0
        divisor        = 1
        profile_z      = hl2ss.depth_profile.ZDEPTH
        buffer_size    = 30 * 10
        sample_time    = 1 / 30

        enable_mrc                 = false
        hologram_composition       = false
        recording_indicator        = false
        video_stabilization        = false
        blank_protected            = false
        show_mesh                  = false
        shared                     = false
        global_opacity             = 0.0
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
        definition_ez
    end

    methods (Access = protected)
        function [image_size] = getImageSize(obj)
            image_size = [obj.height, obj.width];
        end

        function setupImpl(obj)
            obj.definition_ez = ...
                struct('frame_index', zeros([1, 1],             'int64' ), ...
                       'status',      zeros([1, 1],             'int32' ), ...
                       'timestamp',   zeros([1, 1],             'uint64'), ...
                       'depth',       zeros(obj.getImageSize(), 'uint16'));

            coder.extrinsic('hl2ss_matlab')

            obj.client = hl2ss.mt.sink_extended_depth(obj.host, obj.port, obj.media_index, obj.stride_mask, obj.chunk, obj.mode, obj.divisor, obj.profile_z, obj.buffer_size, @hl2ss_matlab);
            
            obj.client.start_subsystem(obj.enable_mrc, obj.hologram_composition, obj.recording_indicator, obj.video_stabilization, obj.blank_protected, obj.show_mesh, obj.shared, obj.global_opacity, obj.output_width, obj.output_height, obj.video_stabilization_length, obj.hologram_perspective)
            obj.client.open()
        end

        function [frame_index, status, timestamp, depth] = stepImpl(obj, sync, index)
            if (sync <= 0)
                response = obj.client.get_packet_by_index(index);
            else
                response = obj.client.get_packet_by_timestamp(sync, obj.time_preference, obj.tiebreak_right);
            end

            coder.extrinsic('hl2ss.ms.unpack_extended_depth')

            data = obj.definition_ez;
            data = hl2ss.ms.unpack_extended_depth(response, obj.mode, obj.getImageSize());

            frame_index = data.frame_index;
            status      = data.status;
            timestamp   = data.timestamp;
            depth       = data.depth;
        end

        function resetImpl(obj)

        end

        function releaseImpl(obj)
            obj.client.close();
            obj.client.stop_subsystem()
        end

        function [out1, out2, out3, out4] = getOutputSizeImpl(obj)
            out1 = [1, 1];
            out2 = [1, 1];
            out3 = [1, 1];
            out4 = obj.getImageSize();
        end

        function [out1, out2, out3, out4] = getOutputDataTypeImpl(obj)
            out1 = 'int64';
            out2 = 'int32';
            out3 = 'uint64';
            out4 = 'uint16';
        end

        function [out1, out2, out3, out4] = isOutputComplexImpl(obj)
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
        end

        function [out1, out2, out3, out4] = isOutputFixedSizeImpl(obj)
            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;
        end

        function sts = getSampleTimeImpl(obj)
            sts = obj.createSampleTime("Type", "Discrete", "SampleTime", obj.sample_time);
        end
    end
end
