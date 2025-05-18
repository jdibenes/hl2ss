
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
    
    function response = ee_get_application_version(self)
        response = self.module('ipc_call', self.port, 'ee_get_application_version');
    end
    
    function response = ts_get_utc_offset(self)
        response = self.module('ipc_call', self.port, 'ts_get_utc_offset');
    end

    function hs_set_marker_state(self, state)
        self.module('ipc_call', self.port, 'hs_set_marker_state', uint32(state));
    end

    function response = pv_get_subsystem_status(self)
        response = self.module('ipc_call', self.port, 'pv_get_subsystem_status');
    end
    
    function pv_set_focus(self, mode, range, distance, value, driver_fallback)
        self.module('ipc_call', self.port, 'pv_set_focus', uint32(mode), uint32(range), uint32(distance), uint32(value), uint32(driver_fallback));
    end

    function pv_set_video_temporal_denoising(self, mode)
        self.module('ipc_call', self.port, 'pv_set_video_temporal_denoising', uint32(mode));
    end

    function pv_set_white_balance_preset(self, preset)
        self.module('ipc_call', self.port, 'pv_set_white_balance_preset', uint32(preset));
    end
    
    function pv_set_white_balance_value(self, value)
        self.module('ipc_call', self.port, 'pv_set_white_balance_value', uint32(value));
    end

    function pv_set_exposure(self, mode, exposure)
        self.module('ipc_call', self.port, 'pv_set_exposure', uint32(mode), uint32(exposure));
    end
    
    function pv_set_exposure_priority_video(self, enabled)
        self.module('ipc_call', self.port, 'pv_set_exposure_priority_video', uint32(enabled));
    end
    
    function pv_set_iso_speed(self, mode, value)
        self.module('ipc_call', self.port, 'pv_set_iso_speed', uint32(mode), uint32(value));
    end

    function pv_set_backlight_compensation(self, state)
        self.module('ipc_call', self.port, 'pv_set_backlight_compensation', uint32(state));
    end

    function pv_set_scene_mode(self, mode)
        self.module('ipc_call', self.port, 'pv_set_scene_mode', uint32(mode));
    end
    
    function ee_set_flat_mode(self, value)
        self.module('ipc_call', self.port, 'ee_set_flat_mode', uint32(value));
    end

    function rm_set_eye_selection(self, enable)
        self.module('ipc_call', self.port, 'rm_set_eye_selection', uint32(enable));
    end

    function pv_set_desired_optimization(self, mode)
        self.module('ipc_call', self.port, 'pv_set_desired_optimization', uint32(mode));
    end

    function pv_set_primary_use(self, mode)
        self.module('ipc_call', self.port, 'pv_set_primary_use', uint32(mode));
    end

    function pv_set_optical_image_stabilization(self, mode)
        self.module('ipc_call', self.port, 'pv_set_optical_image_stabilization', uint32(mode));
    end

    function pv_set_hdr_video(self, mode)
        self.module('ipc_call', self.port, 'pv_set_hdr_video', uint32(mode));
    end

    function pv_set_regions_of_interest(self, clear, set, auto_exposure, auto_focus, bounds_normalized, type, weight, x, y, w, h)
        self.module('ipc_call', self.port, 'pv_set_regions_of_interest', logical(clear), logical(set), logical(auto_exposure), logical(auto_focus), logical(bounds_normalized), uint32(type), uint32(weight), single(x), single(y), single(w), single(h));
    end

    function ee_set_interface_priority(self, port, priority)
        self.module('ipc_call', self.port, 'ee_set_interface_priority', uint16(port), int32(priority));
    end

    function ee_set_quiet_mode(self, mode)
        self.module('ipc_call', self.port, 'ee_set_quiet_mode', uint32(mode));
    end

    function response = rm_map_camera_points(self, port, operation, points)
        response = self.module('ipc_call', self.port, 'rm_map_camera_points', uint16(port), uint32(operation), single(points));
    end

    function response = rm_get_rignode_world_poses(self, timestamps)
        response = pagetranspose(self.module('ipc_call', self.port, 'rm_get_rignode_world_poses', uint64(timestamps)));
    end

    function response = ts_get_current_time(self, source)
        response = self.module('ipc_call', self.port, 'ts_get_current_time', uint32(source));
    end

    function si_set_sampling_delay(self, delay)
        self.module('ipc_call', self.port, 'si_set_sampling_delay', int64(delay));
    end

    function ee_set_encoder_buffering(self, enable)
        self.module('ipc_call', self.port, 'ee_set_encoder_buffering', logical(enable));
    end

    function ee_set_reader_buffering(self, enable)
        self.module('ipc_call', self.port, 'ee_set_reader_buffering', logical(enable));
    end

    function rm_set_loop_control(self, port, enable)
        self.module('ipc_call', self.port, 'rm_set_loop_control', uint16(port), logical(enable));
    end
end
end
