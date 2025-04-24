classdef UavBody < handle
    
    properties (Constant)
        MaxTurnRate = pi/30;
        MinVelocity = 10;
        MaxVelocity = 20;
    end
    
    properties (SetAccess = private, GetAccess = private)
        pos
        bearing
    end
    properties (SetAccess = private, GetAccess = public)
        vel
        turnRate
        operational
        batteryLife
    end
    
    methods
        function uavBody = UavBody(pos, bearing, batteryLife)
            uavBody.pos = pos;
            uavBody.batteryLife = batteryLife;
            uavBody.bearing = bearing;
            uavBody.vel = 15;
            uavBody.turnRate = 0;
            uavBody.operational = true;
        end
        
        function out = outsideMap(uavBody, mapRect)
            out = ...
                uavBody.pos(1) < mapRect(1,1) || ... 
                uavBody.pos(1) > mapRect(2,1) || ...
                uavBody.pos(2) < mapRect(1,2) || ...
                uavBody.pos(2) > mapRect(2,2);
        end
        
        function disable(uavBody)
            uavBody.operational = false;
        end
        
        function move(uavBody, dt)
            if uavBody.operational
                [uavBody.pos, uavBody.bearing] = ...
                    moveRungeKutta( uavBody.pos, uavBody.bearing, uavBody.vel, uavBody.turnRate, dt );
                uavBody.batteryLife = uavBody.batteryLife - dt;
            end
        end
        
        function setVelocity(uavBody, newVel)
            if (newVel > UavBody.MaxVelocity) || (newVel < UavBody.MinVelocity)
                error('Error: UAV velocity out of bounds.');
            end
            uavBody.vel = newVel;
        end
        
        function setTurnRate(uavBody, newTurnRate)
            if (abs(newTurnRate) > UavBody.MaxTurnRate)
                error('Error: UAV turn rate out of bounds.');
            end
            uavBody.turnRate = newTurnRate;
        end
        
        function fuzzPos = getGpsPos(uavBody)
            fuzzAngle = rand * 2 * pi;
            fuzzDist = rand * 3;
            fuzzPos = uavBody.pos + ([sin(fuzzAngle),cos(fuzzAngle)] * fuzzDist);
        end
        
        function conc = sensorReading(uavBody, cloud, t)
            conc = cloudsamp(cloud, uavBody.pos(1), uavBody.pos(2), t);
        end
        
        function plot(uavBody)
            if uavBody.operational
                plot(uavBody.pos(1),uavBody.pos(2),'o');
            else
                plot(uavBody.pos(1),uavBody.pos(2),'ok');
            end
        end
        
    end
    
    methods(Static)
        function colliding = collision(uavA, uavB, dist)
            colliding = (sqrLen(uavA.pos - uavB.pos) <= dist^2);
        end
    end
    
end

