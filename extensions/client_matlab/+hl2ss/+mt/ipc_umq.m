
classdef ipc_umq
properties
    host
    port
    module
end

methods
    function self = ipc_umq(host, port, module)
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
    
    function push(self, data)
        self.module('ipc_call', self.port, 'push', data);
    end
    
    function response = pull(self, count)
        response = self.module('ipc_call', self.port, 'pull', uint32(count));
    end
end
end
