
classdef sink_microphone
properties
    host
    port
    chunk
    profile
    level
    buffer_size
    module
end

methods
    function self = sink_microphone(host, port, chunk, profile, level, buffer_size, module)
        arguments
            host
            port
            chunk       = 4096
            profile     = hl2ss.audio_profile.AAC_24000
            level       = hl2ss.aac_level.L2
            buffer_size = 1000
            module      = @hl2ss_matlab
        end
        
        self.host        = host;
        self.port        = uint16(port);
        self.chunk       = uint64(chunk);
        self.profile     = uint8(profile);
        self.level       = uint8(level);
        self.buffer_size = uint64(buffer_size);
        self.module      = module;
    end
    
    function open(self)
        self.module('open', self.host, self.port, self.chunk, self.profile, self.level, self.buffer_size);
    end
    
    function response = get_packet_by_index(self, index)
        response = self.module('get_packet', self.port, hl2ss.grab_mode.BY_FRAME_INDEX, int64(index));
    end
    
    function response = get_packet_by_timestamp(self, timestamp, time_preference, tiebreak_right)
        response = self.module('get_packet', self.port, hl2ss.grab_mode.BY_TIMESTAMP, uint64(timestamp), int32(time_preference), int32(tiebreak_right));
    end
    
    function close(self)
        self.module('close', self.port);
    end
end
end
