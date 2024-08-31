
classdef sink_eet < matlab.System
    properties
    
    end

    properties (Nontunable)
        host        = '192.168.1.7'
        port        = hl2ss.stream_port.EXTENDED_EYE_TRACKER
        chunk       = 4096
        framerate   = 90
        buffer_size = 300
        sample_time = 1 / 90

        time_preference = hl2ss.grab_preference.PREFER_NEAREST
        tiebreak_right  = false
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        client
        definition_eet
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.definition_eet = ...
                struct('frame_index',       zeros([1, 1], 'int64' ), ...
                       'status',            zeros([1, 1], 'int32' ), ...
                       'timestamp',         zeros([1, 1], 'uint64'), ...
                       'combined_ray',      zeros([6, 1], 'single'), ...
                       'left_ray',          zeros([6, 1], 'single'), ...
                       'right_ray',         zeros([6, 1], 'single'), ...
                       'left_openness',     zeros([1, 1], 'single'), ...
                       'right_openness',    zeros([1, 1], 'single'), ...
                       'vergence_distance', zeros([1, 1], 'single'), ...
                       'valid',             zeros([1, 1], 'uint32'), ...
                       'pose',              zeros([4, 4], 'single'));

            coder.extrinsic('hl2ss_matlab')

            obj.client = hl2ss.mt.sink_eet(obj.host, obj.port, obj.chunk, obj.framerate, obj.buffer_size, @hl2ss_matlab);
            
            obj.client.open()
        end

        function [frame_index, status, timestamp, combined_ray, left_ray, right_ray, left_openness, right_openness, vergence_distance, valid, pose] = stepImpl(obj, sync, index)
            if (sync <= 0)
                response = obj.client.get_packet_by_index(index);
            else
                response = obj.client.get_packet_by_timestamp(sync, obj.time_preference, obj.tiebreak_right);
            end

            coder.extrinsic('hl2ss.ms.unpack_eet')

            data = obj.definition_eet;
            data = hl2ss.ms.unpack_eet(response);

            frame_index       = data.frame_index;
            status            = data.status;
            timestamp         = data.timestamp;
            combined_ray      = data.combined_ray;
            left_ray          = data.left_ray;
            right_ray         = data.right_ray;
            left_openness     = data.left_openness;
            right_openness    = data.right_openness;
            vergence_distance = data.vergence_distance;
            valid             = data.valid;
            pose              = data.pose;
        end

        function resetImpl(obj)

        end

        function releaseImpl(obj)
            obj.client.close();
        end

        function [out01, out02, out03, out04, out05, out06, out07, out08, out09, out10, out11] = getOutputSizeImpl(obj)
            out01 = [1, 1];
            out02 = [1, 1];
            out03 = [1, 1];
            out04 = [6, 1];
            out05 = [6, 1];
            out06 = [6, 1];
            out07 = [1, 1];
            out08 = [1, 1];
            out09 = [1, 1];
            out10 = [1, 1];
            out11 = [4, 4];
        end

        function [out01, out02, out03, out04, out05, out06, out07, out08, out09, out10, out11] = getOutputDataTypeImpl(obj)
            out01 = 'int64';
            out02 = 'int32';
            out03 = 'uint64';
            out04 = 'single';
            out05 = 'single';
            out06 = 'single';
            out07 = 'single';
            out08 = 'single';
            out09 = 'single';
            out10 = 'uint32';
            out11 = 'single';
        end

        function [out01, out02, out03, out04, out05, out06, out07, out08, out09, out10, out11] = isOutputComplexImpl(obj)
            out01 = false;
            out02 = false;
            out03 = false;
            out04 = false;
            out05 = false;
            out06 = false;
            out07 = false;
            out08 = false;
            out09 = false;
            out10 = false;
            out11 = false;
        end

        function [out01, out02, out03, out04, out05, out06, out07, out08, out09, out10, out11] = isOutputFixedSizeImpl(obj)
            out01 = true;
            out02 = true;
            out03 = true;
            out04 = true;
            out05 = true;
            out06 = true;
            out07 = true;
            out08 = true;
            out09 = true;
            out10 = true;
            out11 = true;
        end

        function sts = getSampleTimeImpl(obj)
            sts = obj.createSampleTime("Type", "Discrete", "SampleTime", obj.sample_time);
        end
    end
end
