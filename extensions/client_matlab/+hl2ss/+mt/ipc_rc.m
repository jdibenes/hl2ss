
classdef ipc_rc
properties
    host
    port
    module
end    
    
methods
    function self = ipc_rc(host, port, module)
        arguments
            host
            port
            module = @hl2ss_matlab;
        end
        
        self.host   = host;
        self.port   = uint16(port);
        self.module = module;
    end
    
    function open(self)
        self.module('open', self.host, self.port);
    end
    
    function close(self)
        self.module('close', self.port);
    end
    
    function response = get_application_version(self)
        response = self.module('ipc_call', self.port, 'get_application_version');
    end
    
    function response = get_pv_subsystem_status(self)
        response = self.module('ipc_call', self.port, 'get_pv_subsystem_status');
    end
    
    function response = get_utc_offset(self, samples)
        response = self.module('ipc_call', self.port, 'get_utc_offset', uint32(samples));
    end
    
    function set_hs_marker_state(self, state)
        self.module('ipc_call', self.port, 'set_hs_marker_state', uint32(state));
    end
    
    function set_pv_backlight_compensation(self, state)
        self.module('ipc_call', self.port, 'set_pv_backlight_compensation', uint32(state));
    end
    
    function set_pv_exposure(self, mode, exposure)
        self.module('ipc_call', self.port, 'set_pv_exposure', uint32(mode), uint32(exposure));
    end
    
    function set_pv_exposure_priority_video(self, enabled)
        self.module('ipc_call', self.port, 'set_pv_exposure_priority_video', uint32(enabled));
    end
    
    function set_pv_focus(self, mode, range, distance, value, driver_fallback)
        self.module('ipc_call', self.port, 'set_pv_focus', uint32(mode), uint32(range), uint32(distance), uint32(value), uint32(driver_fallback));
    end
    
    function set_pv_iso_speed(self, mode, value)
        self.module('ipc_call', self.port, 'set_pv_iso_speed', uint32(mode), uint32(value));
    end
    
    function set_pv_scene_mode(self, mode)
        self.module('ipc_call', self.port, 'set_pv_scene_mode', uint32(mode));
    end
    
    function set_pv_video_temporal_denoising(self, mode)
        self.module('ipc_call', self.port, 'set_pv_video_temporal_denoising', uint32(mode));
    end
    
    function set_pv_white_balance_preset(self, preset)
        self.module('ipc_call', self.port, 'set_pv_white_balance_preset', uint32(preset));
    end
    
    function set_pv_white_balance_value(self, value)
        self.module('ipc_call', self.port, 'set_pv_white_balance_value', uint32(value));
    end

    function set_flat_mode(self, value)
        self.module('ipc_call', self.port, 'set_flat_mode', uint32(value));
    end
end
end
