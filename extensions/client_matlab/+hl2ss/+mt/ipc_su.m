
classdef ipc_su
properties
    host
    port
    module
end

methods
    function self = ipc_su(host, port, module)
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
    
    function response = query(self, task)
        task.enable_quads         = logical(task.enable_quads);
        task.enable_meshes        = logical(task.enable_meshes);
        task.enable_only_observed = logical(task.enable_only_observed);
        task.enable_world_mesh    = logical(task.enable_world_mesh);
        task.mesh_lod             = uint32(task.mesh_lod);
        task.query_radius         = single(task.query_radius);
        task.create_mode          = uint8(task.create_mode);
        task.kind_flags           = uint8(task.kind_flags);
        task.get_orientation      = logical(task.get_orientation);
        task.get_position         = logical(task.get_position);
        task.get_location_matrix  = logical(task.get_location_matrix);
        task.get_quad             = logical(task.get_quad);
        task.get_meshes           = logical(task.get_meshes);
        task.get_collider_meshes  = logical(task.get_collider_meshes);
        task.guid_list            = uint64(task.guid_list);
        
        response = self.module('ipc_call', self.port, 'query', task);
    end
end
end
