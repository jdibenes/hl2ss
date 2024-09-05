
classdef sink_rm_vlc < matlab.System
    properties
    
    end

    properties (Nontunable)
        host        = '192.168.1.7'
        port        = hl2ss.stream_port.RM_VLC_LEFTFRONT
        chunk       = 4096
        mode        = hl2ss.stream_mode.MODE_1
        divisor     = 1
        profile     = hl2ss.video_profile.H265_MAIN
        level       = hl2ss.h26x_level.DEFAULT
        bitrate     = 0
        options     = [hl2ss.h26x_encoder_property.CODECAPI_AVEncMPVGOPSize, hl2ss.parameters_rm_vlc.FPS, hl2ss.h26x_encoder_property.HL2SSAPI_VLCHostTicksOffsetExposure, 0, hl2ss.h26x_encoder_property.HL2SSAPI_VLCHostTicksOffsetConstant, 0xFFFFFFFFFFFE17B8]
        buffer_size = hl2ss.parameters_rm_vlc.FPS * 10
        sample_time = 1 / hl2ss.parameters_rm_vlc.FPS

        time_preference = hl2ss.grab_preference.PREFER_NEAREST
        tiebreak_right  = false
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        client
        definition_rm_vlc
    end

    methods (Access = protected)
        function [image_size] = getImageSize(obj)
            image_size = [hl2ss.parameters_rm_vlc.HEIGHT, hl2ss.parameters_rm_vlc.WIDTH];
        end

        function setupImpl(obj)
            obj.definition_rm_vlc = ...
                struct('frame_index',  zeros([1, 1],             'int64' ), ...
                       'status',       zeros([1, 1],             'int32' ), ...
                       'timestamp',    zeros([1, 1],             'uint64'), ...
                       'image',        zeros(obj.getImageSize(), 'uint8' ), ...
                       'sensor_ticks', zeros([1, 1],             'uint64'), ...
                       'exposure',     zeros([1, 1],             'uint64'), ...
                       'gain',         zeros([1, 1],             'uint32'), ...
                       'pose',         zeros([4, 4],             'single'));

            coder.extrinsic('hl2ss_matlab')

            obj.client = hl2ss.mt.sink_rm_vlc(obj.host, obj.port, obj.chunk, obj.mode, obj.divisor, obj.profile, obj.level, obj.bitrate, obj.options, obj.buffer_size, @hl2ss_matlab);
            
            obj.client.open()
        end

        function [frame_index, status, timestamp, image, pose, sensor_ticks, exposure, gain] = stepImpl(obj, sync, index)
            if (sync <= 0)
                response = obj.client.get_packet_by_index(index);
            else
                response = obj.client.get_packet_by_timestamp(sync, obj.time_preference, obj.tiebreak_right);
            end
            
            coder.extrinsic('hl2ss.ms.unpack_rm_vlc')

            data = obj.definition_rm_vlc;
            data = hl2ss.ms.unpack_rm_vlc(response, obj.mode);

            frame_index  = data.frame_index;
            status       = data.status;
            timestamp    = data.timestamp;
            image        = data.image;
            sensor_ticks = data.sensor_ticks;
            exposure     = data.exposure;
            gain         = data.gain;
            pose         = data.pose;
        end

        function resetImpl(obj)

        end

        function releaseImpl(obj)
            obj.client.close();
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8] = getOutputSizeImpl(obj)
            out1 = [1, 1];
            out2 = [1, 1];
            out3 = [1, 1];
            out4 = obj.getImageSize();
            out5 = [4, 4];
            out6 = [1, 1];
            out7 = [1, 1];
            out8 = [1, 1];
        end

        function [out1, out2, out3, out4, out5, out6, out7, out8] = getOutputDataTypeImpl(obj)
            out1 = 'int64';
            out2 = 'int32';
            out3 = 'uint64';
            out4 = 'uint8';
            out5 = 'single';
            out6 = 'uint64';
            out7 = 'uint64';
            out8 = 'uint32';
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
