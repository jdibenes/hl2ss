
classdef sink_extended_audio
properties
    host
    port
    chunk
    mixer_mode
    loopback_gain
    microphone_gain
    profile
    level
    buffer_size
    module
end

methods
    function self = sink_extended_audio(host, port, chunk, mixer_mode, loopback_gain, microphone_gain, profile, level, buffer_size, module)
        arguments
            host
            port
            chunk           = 4096
            mixer_mode      = hl2ss.mixer_mode.BOTH
            loopback_gain   = 1.0
            microphone_gain = 1.0
            profile         = hl2ss.audio_profile.AAC_24000
            level           = hl2ss.aac_level.L2
            buffer_size     = 1000
            module          = @hl2ss_matlab;
        end
        
        self.host            = host;
        self.port            = uint16(port);
        self.chunk           = uint64(chunk);
        self.mixer_mode      = uint32(mixer_mode);
        self.loopback_gain   = single(loopback_gain);
        self.microphone_gain = single(microphone_gain);
        self.profile         = uint8(profile);
        self.level           = uint8(level);
        self.buffer_size     = uint64(buffer_size);
        self.module          = module;
    end
    
    function open(self)
        self.module('open', self.host, self.port, self.chunk, self.mixer_mode, self.loopback_gain, self.microphone_gain, self.profile, self.level, self.buffer_size);
    end
    
    function response = get_packet_by_index(self, index)
        response = self.module('get_packet', self.port, hl2ss.grab_mode.BY_FRAME_INDEX, int64(index));
    end
    
    function response = get_packet_by_timestamp(self, timestamp, preference)
        response = self.module('get_packet', self.port, hl2ss.grab_mode.BY_TIMESTAMP, uint64(timestamp), int32(preference));
    end
    
    function close(self)
        self.module('close', self.port);
    end
end
end
