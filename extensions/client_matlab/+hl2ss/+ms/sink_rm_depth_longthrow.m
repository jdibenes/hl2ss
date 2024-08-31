
classdef sink_rm_depth_longthrow < matlab.System
    properties
    
    end

    properties (Nontunable)
        host        = '192.168.1.7'
        port        = hl2ss.stream_port.RM_DEPTH_LONGTHROW
        chunk       = 4096
        mode        = hl2ss.stream_mode.MODE_1
        divisor     = 1
        png_filter  = hl2ss.png_filter_mode.PAETH
        buffer_size = hl2ss.parameters_rm_depth_longthrow.FPS * 10
        sample_time = 1 / hl2ss.parameters_rm_depth_longthrow.FPS

        time_preference = hl2ss.grab_preference.PREFER_NEAREST
        tiebreak_right  = false
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        client
        definition_rm_depth_longthrow
    end

    methods (Access = protected)
        function [image_size] = getImageSize(obj)
            image_size = [hl2ss.parameters_rm_depth_longthrow.HEIGHT, hl2ss.parameters_rm_depth_longthrow.WIDTH];
        end

        function setupImpl(obj)
            obj.definition_rm_depth_longthrow = ...
                struct('frame_index',  zeros([1, 1],             'int64' ), ...
                       'status',       zeros([1, 1],             'int32' ), ...
                       'timestamp',    zeros([1, 1],             'uint64'), ...
                       'depth',        zeros(obj.getImageSize(), 'uint16'), ...
                       'ab',           zeros(obj.getImageSize(), 'uint16'), ...
                       'sensor_ticks', zeros([1, 1],             'uint64'), ...
                       'pose',         zeros([4, 4],             'single'));

            coder.extrinsic('hl2ss_matlab')

            obj.client = hl2ss.mt.sink_rm_depth_longthrow(obj.host, obj.port, obj.chunk, obj.mode, obj.divisor, obj.png_filter, obj.buffer_size, @hl2ss_matlab);
            
            obj.client.open()
        end

        function [frame_index, status, timestamp, depth, ab, pose, sensor_ticks] = stepImpl(obj, sync, index)
            if (sync <= 0)
                response = obj.client.get_packet_by_index(index);
            else
                response = obj.client.get_packet_by_timestamp(sync, obj.time_preference, obj.tiebreak_right);
            end

            coder.extrinsic('hl2ss.ms.unpack_rm_depth_longthrow')

            data = obj.definition_rm_depth_longthrow;
            data = hl2ss.ms.unpack_rm_depth_longthrow(response, obj.mode);

            frame_index  = data.frame_index;
            status       = data.status;
            timestamp    = data.timestamp;
            depth        = data.depth;
            ab           = data.ab;
            sensor_ticks = data.sensor_ticks;
            pose         = data.pose;
        end

        function resetImpl(obj)

        end

        function releaseImpl(obj)
            obj.client.close();
        end

        function [out1, out2, out3, out4, out5, out6, out7] = getOutputSizeImpl(obj)
            out1 = [1, 1];
            out2 = [1, 1];
            out3 = [1, 1];
            out4 = obj.getImageSize();
            out5 = obj.getImageSize();
            out6 = [4, 4];
            out7 = [1, 1];
        end

        function [out1, out2, out3, out4, out5, out6, out7] = getOutputDataTypeImpl(obj)
            out1 = 'int64';
            out2 = 'int32';
            out3 = 'uint64';
            out4 = 'uint16';
            out5 = 'uint16';
            out6 = 'single';
            out7 = 'uint64';
        end

        function [out1, out2, out3, out4, out5, out6, out7] = isOutputComplexImpl(obj)
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
            out7 = false;
        end

        function [out1, out2, out3, out4, out5, out6, out7] = isOutputFixedSizeImpl(obj)
            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
            out6 = true;
            out7 = true;
        end

        function sts = getSampleTimeImpl(obj)
            sts = obj.createSampleTime("Type", "Discrete", "SampleTime", obj.sample_time);
        end
    end
end
