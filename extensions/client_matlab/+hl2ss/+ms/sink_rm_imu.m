
classdef sink_rm_imu < matlab.System
    properties
    
    end

    properties (Nontunable)
        host        = '192.168.1.7'
        port        = hl2ss.stream_port.RM_IMU_ACCELEROMETER
        chunk       = 4096
        mode        = hl2ss.stream_mode.MODE_1
        buffer_size = 300
        sample_time = 1 / 30

        time_preference = hl2ss.grab_preference.PREFER_NEAREST
        tiebreak_right  = false
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        client
        definition_rm_imu
    end

    methods (Access = protected)
        function [image_size] = getImageSize(obj)
            if     (obj.port == hl2ss.stream_port.RM_IMU_ACCELEROMETER)
                image_size = [hl2ss.parameters_rm_imu_accelerometer.BATCH_SIZE, 1];
            elseif (obj.port == hl2ss.stream_port.RM_IMU_GYROSCOPE)
                image_size = [hl2ss.parameters_rm_imu_gyroscope.BATCH_SIZE, 1];
            elseif (obj.port == hl2ss.stream_port.RM_IMU_MAGNETOMETER)
                image_size = [hl2ss.parameters_rm_imu_magnetometer.BATCH_SIZE, 1];
            end
        end

        function setupImpl(obj)
            obj.definition_rm_imu = ...
                struct('frame_index',      zeros([1, 1],             'int64' ), ...
                       'status',           zeros([1, 1],             'int32' ), ...
                       'timestamp',        zeros([1, 1],             'uint64'), ...
                       'sensor_timestamp', zeros(obj.getImageSize(), 'uint64'), ...
                       'host_timestamp',   zeros(obj.getImageSize(), 'uint64'), ...
                       'x',                zeros(obj.getImageSize(), 'single'), ...
                       'y',                zeros(obj.getImageSize(), 'single'), ...
                       'z',                zeros(obj.getImageSize(), 'single'), ...
                       'temperature',      zeros(obj.getImageSize(), 'single'), ...
                       'pose',             zeros([4, 4],             'single'));

            coder.extrinsic('hl2ss_matlab')

            obj.client = hl2ss.mt.sink_rm_imu(obj.host, obj.port, obj.chunk, obj.mode, obj.buffer_size, @hl2ss_matlab);
            
            obj.client.open()
        end

        function [frame_index, status, timestamp, sensor_timestamp, host_timestamp, x, y, z, temperature, pose] = stepImpl(obj, sync, index)
            if (sync <= 0)
                response = obj.client.get_packet_by_index(index);
            else
                response = obj.client.get_packet_by_timestamp(sync, obj.time_preference, obj.tiebreak_right);
            end

            coder.extrinsic('hl2ss.ms.unpack_rm_imu')

            data = obj.definition_rm_imu;
            data = hl2ss.ms.unpack_rm_imu(response, obj.mode, obj.getImageSize());

            frame_index      = data.frame_index;
            status           = data.status;
            timestamp        = data.timestamp;
            sensor_timestamp = data.sensor_timestamp;
            host_timestamp   = data.host_timestamp;
            x                = data.x;
            y                = data.y;
            z                = data.z;
            temperature      = data.temperature;
            pose             = data.pose;
        end

        function resetImpl(obj)

        end

        function releaseImpl(obj)
            obj.client.close();
        end

        function [out01, out02, out03, out04, out05, out06, out07, out08, out09, out10] = getOutputSizeImpl(obj)
            out01 = [1, 1];
            out02 = [1, 1];
            out03 = [1, 1];
            out04 = obj.getImageSize();
            out05 = obj.getImageSize();
            out06 = obj.getImageSize();
            out07 = obj.getImageSize();
            out08 = obj.getImageSize();
            out09 = obj.getImageSize();
            out10 = [4, 4];
        end

        function [out01, out02, out03, out04, out05, out06, out07, out08, out09, out10] = getOutputDataTypeImpl(obj)
            out01 = 'int64';
            out02 = 'int32';
            out03 = 'uint64';
            out04 = 'uint64';
            out05 = 'uint64';
            out06 = 'single';
            out07 = 'single';
            out08 = 'single';
            out09 = 'single';
            out10 = 'single';
        end

        function [out01, out02, out03, out04, out05, out06, out07, out08, out09, out10] = isOutputComplexImpl(obj)
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
        end

        function [out01, out02, out03, out04, out05, out06, out07, out08, out09, out10] = isOutputFixedSizeImpl(obj)
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
        end

        function sts = getSampleTimeImpl(obj)
            sts = obj.createSampleTime("Type", "Discrete", "SampleTime", obj.sample_time);
        end
    end
end
