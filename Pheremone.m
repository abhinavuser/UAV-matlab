classdef Pheremone
    % Class used to represent pseudo-pheremones used by UAVs
    
    properties (Constant)
        DefaultDuration = 60.0;
    end    
    
    properties (SetAccess = public, GetAccess = public)
        pos;
        duration;
        type;
    end
    
    methods
        function pheremone = Pheremone(pos, type)
            pheremone.pos = pos;
            pheremone.duration = Pheremone.DefaultDuration;
            pheremone.type = type;
        end
    end
    
end

