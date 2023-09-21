
classdef umq_command_buffer < handle
properties
    data
    count
end

methods
    function self = umq_command_buffer()
        self.data  = uint8([]);
        self.count = uint32(0);
    end
    
    function add(self, command, parameters)
        id         = typecast(uint32(command), 'uint8');
        parameters = uint8(parameters);
        length     = typecast(uint32(numel(parameters)), 'uint8');
        
        self.data  = [self.data; [id(:); length(:); parameters(:)]];
        self.count = self.count + 1;
    end
end
end
