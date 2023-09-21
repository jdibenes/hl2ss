
classdef ipc_vi
properties
    host
    port
    module
end

methods
    function self = ipc_vi(host, port, module)
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
    
    function create_recognizer(self)
        self.module('ipc_call', self.port, 'create_recognizer');
    end
    
    function response = register_commands(self, clear, commands)
        response = self.module('ipc_call', self.port, 'register_commands', logical(clear), commands);
    end
    
    function start(self)
        self.module('ipc_call', self.port, 'start');
    end
    
    function clear(self)
        self.module('ipc_call', self.port, 'clear');
    end
    
    function response = pop(self)
        response = self.module('ipc_call', self.port, 'pop');
    end
    
    function stop(self)
        self.module('ipc_call', self.port, 'stop');
    end
end
end
