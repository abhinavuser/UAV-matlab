function [ move, turn ] = forwardEuler( bearing, speed, turnRate )
    move = speed * [sin(bearing) cos(bearing)];
    turn = turnRate * speed;
end

