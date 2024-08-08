
classdef ipc_gmq
properties
    host
    port
    module
end

methods
    function self = ipc_gmq(host, port, module)
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

    function [command, data] = pull(self)
        [command, data] = self.module('ipc_call', self.port, 'pull');
    end
    
    function push(self, response)
        self.module('ipc_call', self.port, 'push', uint32(response));
    end
end
end
