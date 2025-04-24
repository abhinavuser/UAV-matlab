classdef UavState < uint8
    enumeration
        Inactive    (0)
        Start       (1)
        Calibration (2)
        Search      (3)
        Approach    (4)
        Track       (5)
        Land        (6)
    end
    
    methods(Static)
        % Gives the avoidance priority between two states:
        % Returns 1 if stateA has greater priority than stateB, -1 if it
        % has lower, and 0 if they are equal
        function result = priority(stateA, stateB)
            if stateA == stateB
                result = 0;
            elseif stateA < stateB
                result = -1;
            else
                result = 1;
            end
        end
    end
end

