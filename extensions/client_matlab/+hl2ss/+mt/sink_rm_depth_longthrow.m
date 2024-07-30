
classdef sink_rm_depth_longthrow
properties
    host
    port
    chunk
    mode
    divisor
    png_filter
    buffer_size
    module
end

methods
    function self = sink_rm_depth_longthrow(host, port, chunk, mode, divisor, png_filter, buffer_size, module)
        arguments
            host
            port
            chunk       = 4096
            mode        = hl2ss.stream_mode.MODE_1
            divisor     = 1
            png_filter  = hl2ss.png_filter_mode.PAETH
            buffer_size = 50
            module      = @hl2ss_matlab
        end
        
        self.host        = host;
        self.port        = uint16(port);
        self.chunk       = uint64(chunk);
        self.mode        = uint8(mode);
        self.divisor     = uint8(divisor);
        self.png_filter  = uint8(png_filter);
        self.buffer_size = uint64(buffer_size);
        self.module      = module;
    end
    
    function open(self)
        self.module('open', self.host, self.port, self.chunk, self.mode, self.divisor, self.png_filter, self.buffer_size);
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
    
    function response = download_calibration(self)
        response = self.module('download_calibration', self.host, self.port);
    end
end
end
