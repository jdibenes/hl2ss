
classdef sink_extended_audio < matlab.System
    properties
    
    end

    properties (Nontunable)
        host            = '192.168.1.7'
        port            = hl2ss.stream_port.EXTENDED_AUDIO
        chunk           = 4096
        mixer_mode      = hl2ss.mixer_mode.BOTH
        device          = -1
        loopback_gain   = 1.0
        microphone_gain = 1.0
        profile         = hl2ss.audio_profile.AAC_24000
        level           = hl2ss.aac_level.L2
        buffer_size     = 1000
        sample_time     = hl2ss.parameters_microphone.GROUP_SIZE_AAC / hl2ss.parameters_microphone.SAMPLE_RATE

        time_preference = hl2ss.grab_preference.PREFER_NEAREST
        tiebreak_right  = false
    end

    properties (DiscreteState)

    end

    % Pre-computed constants
    properties (Access = private)
        client
        definition_microphone
    end

    methods (Access = protected)
        function [audio_size, audio_type] = getAudioType(obj)
            % hl2ss.audio_profile.RAW is not supported 
            if (obj.profile ~= hl2ss.audio_profile.RAW)
                audio_size = [hl2ss.parameters_microphone.CHANNELS, hl2ss.parameters_microphone.GROUP_SIZE_AAC];
                audio_type = 'single';
            end
        end

        function setupImpl(obj)
            [audio_size, audio_type] = obj.getAudioType();

            obj.definition_microphone = ...
                struct('frame_index', zeros([1, 1],     'int64' ), ...
                       'status',      zeros([1, 1],     'int32' ), ...
                       'timestamp',   zeros([1, 1],     'uint64'), ...
                       'audio',       zeros(audio_size, audio_type));

            coder.extrinsic('hl2ss_matlab')

            obj.client = hl2ss.mt.sink_extended_audio(obj.host, obj.port, obj.chunk, hl2ss.extended_audio_device_mixer_mode(obj.mixer_mode, obj.device), obj.loopback_gain, obj.microphone_gain, obj.profile, obj.level, obj.buffer_size, @hl2ss_matlab);
            
            obj.client.open()
        end

        function [frame_index, status, timestamp, audio] = stepImpl(obj, sync, index)
            [audio_size, audio_type] = obj.getAudioType();

            if (sync <= 0)
                response = obj.client.get_packet_by_index(index);
            else
                response = obj.client.get_packet_by_timestamp(sync, obj.time_preference, obj.tiebreak_right);
            end

            coder.extrinsic('hl2ss.ms.unpack_microphone')

            data = obj.definition_microphone;
            data = hl2ss.ms.unpack_microphone(response, audio_size, audio_type);

            frame_index = data.frame_index;
            status      = data.status;
            timestamp   = data.timestamp;
            audio       = data.audio;
        end

        function resetImpl(obj)

        end

        function releaseImpl(obj)
            obj.client.close();
        end

        function [out1, out2, out3, out4] = getOutputSizeImpl(obj)
            [audio_size, ~] = obj.getAudioType();

            out1 = [1, 1];
            out2 = [1, 1];
            out3 = [1, 1];
            out4 = audio_size;
        end

        function [out1, out2, out3, out4] = getOutputDataTypeImpl(obj)
            [~, audio_type] = obj.getAudioType();

            out1 = 'int64';
            out2 = 'int32';
            out3 = 'uint64';
            out4 = audio_type;
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
