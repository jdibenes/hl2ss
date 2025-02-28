
classdef sink_extended_depth
properties
    host
    port
    media_index
    stride_mask
    chunk
    mode
    divisor
    profile_z
    buffer_size
    module
end

methods
    function self = sink_extended_depth(host, port, media_index, stride_mask, chunk, mode, divisor, profile_z, buffer_size, module)
        arguments
            host
            port
            media_index     = 0xFFFFFFFF
            stride_mask     = 63
            chunk           = 4096
            mode            = 0
            divisor         = 1
            profile_z       = hl2ss.depth_profile.ZDEPTH
            buffer_size     = 300
            module          = @hl2ss_matlab;
        end
        
        self.host            = host;
        self.port            = uint16(port);
        self.media_index     = uint64(media_index);
        self.stride_mask     = uint64(stride_mask);
        self.chunk           = uint64(chunk);
        self.mode            = uint8(mode);
        self.divisor         = uint8(divisor);
        self.profile_z       = uint8(profile_z);
        self.buffer_size     = uint64(buffer_size);
        self.module          = module;
    end
    
    function open(self)
        self.module('open', self.host, self.port, self.chunk, self.media_index, self.stride_mask, self.mode, self.divisor, self.profile_z, self.buffer_size);
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

    function start_subsystem(self, enable_mrc, hologram_composition, recording_indicator, video_stabilization, blank_protected, show_mesh, shared, global_opacity, output_width, output_height, video_stabilization_length, hologram_perspective)
        arguments
            self
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
        end
    
        self.module('start_subsystem_pv', self.host, self.port, logical(enable_mrc), logical(hologram_composition), logical(recording_indicator), logical(video_stabilization), logical(blank_protected), logical(show_mesh), logical(shared), single(global_opacity), single(output_width), single(output_height), uint32(video_stabilization_length), uint32(hologram_perspective));
    end

    function stop_subsystem(self)
        self.module('stop_subsystem_pv', self.host, self.port);
    end
end
end
