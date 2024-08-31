
classdef sink_si < matlab.System
    properties
    
    end

    properties (Nontunable)
        host        = '192.168.1.7'
        port        = hl2ss.stream_port.SPATIAL_INPUT
        chunk       = 4096
        buffer_size = 300
        sample_time = 1 / hl2ss.parameters_si.SAMPLE_RATE

        time_preference = hl2ss.grab_preference.PREFER_NEAREST
        tiebreak_right  = false
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        client
        definition_si
    end

    methods (Access = protected)
        function [head_pose_size, eye_ray_size, hand_size] = getDataSize(obj)
            head_pose_size = [3,  3];
            eye_ray_size   = [6,  1];
            hand_size      = [9, 26];
        end

        function setupImpl(obj)
            [head_pose_size, eye_ray_size, hand_size] = obj.getDataSize();

            obj.definition_si = ...
                struct('frame_index', zeros([1, 1],         'int64' ), ...
                       'status',      zeros([1, 1],         'int32' ), ...
                       'timestamp',   zeros([1, 1],         'uint64'), ...
                       'valid',       zeros([1, 1],         'uint32'), ...
                       'head_pose',   zeros(head_pose_size, 'single'), ...
                       'eye_ray',     zeros(eye_ray_size,   'single'), ...
                       'left_hand',   zeros(hand_size,      'single'), ...
                       'right_hand',  zeros(hand_size,      'single'));

            coder.extrinsic('hl2ss_matlab')

            obj.client = hl2ss.mt.sink_si(obj.host, obj.port, obj.chunk, obj.buffer_size, @hl2ss_matlab);
            
            obj.client.open()
        end

        function [frame_index, status, timestamp, valid, head_pose, eye_ray, left_hand, right_hand] = stepImpl(obj, sync, index)
            if (sync <= 0)
                response = obj.client.get_packet_by_index(index);
            else
                response = obj.client.get_packet_by_timestamp(sync, obj.time_preference, obj.tiebreak_right);
            end

            coder.extrinsic('hl2ss.ms.unpack_si')

            data = obj.definition_si;
            data = hl2ss.ms.unpack_si(response);

            frame_index = data.frame_index;
            status      = data.status;
            timestamp   = data.timestamp;
            valid       = data.valid;
            head_pose   = data.head_pose;
            eye_ray     = data.eye_ray;
            left_hand   = data.left_hand;
            right_hand  = data.right_hand;
        end

        function resetImpl(obj)

        end

        function releaseImpl(obj)
            obj.client.close();
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8] = getOutputSizeImpl(obj)
            [head_pose_size, eye_ray_size, hand_size] = obj.getDataSize();

            out1 = [1, 1];
            out2 = [1, 1];
            out3 = [1, 1];
            out4 = [1, 1];
            out5 = head_pose_size;
            out6 = eye_ray_size;
            out7 = hand_size;
            out8 = hand_size;
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8] = getOutputDataTypeImpl(obj)
            out1 = 'int64';
            out2 = 'int32';
            out3 = 'uint64';
            out4 = 'uint32';
            out5 = 'single';
            out6 = 'single';
            out7 = 'single';
            out8 = 'single';
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8] = isOutputComplexImpl(obj)
            out1 = false;
            out2 = false;
            out3 = false;
            out4 = false;
            out5 = false;
            out6 = false;
            out7 = false;
            out8 = false;
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8] = isOutputFixedSizeImpl(obj)
            out1 = true;
            out2 = true;
            out3 = true;
            out4 = true;
            out5 = true;
            out6 = true;
            out7 = true;
            out8 = true;
        end

        function sts = getSampleTimeImpl(obj)
            sts = obj.createSampleTime("Type", "Discrete", "SampleTime", obj.sample_time);
        end
    end
end
