
classdef ipc_sm
properties
    host
    port
    module
end

methods
    function self = ipc_sm(host, port, module)
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
        
    function set_volumes(self, volumes)
        self.module('ipc_call', self.port, 'set_volumes', volumes);
    end
    
    function response = get_observed_surfaces(self)
        response = self.module('ipc_call', self.port, 'get_observed_surfaces');
    end
    
    function response = get_meshes(self, tasks, threads)
        response = self.module('ipc_call', self.port, 'get_meshes', tasks, uint32(threads));
    end
end
end
