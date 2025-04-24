classdef Message
    % Message sent by a UAV; no larger than 32 bytes
    %   pos is the perceived [x,y] coordinate of the UAV
    %   bearing is the perceived bearing of the UAV
    %   state is the FSM state of the UAV
    %   pheremoneType indicates whether the message contains a pheremone
    %   id is the ID of the UAV
    properties
        % 16 bytes
        pos;
        % 8 bytes
        bearing;
        % 1 byte
        state;
        % 1 byte
        pheremoneType;
        % 1 byte
        id;
    end    
    
    methods
        function message = Message()
            message.pos = [0,0];
            message.id = uint8(-1);
            message.bearing = 0;
            message.state = UavState.Inactive;
            message.pheremoneType = PheremoneType.None;
        end
    end
end

