function [ pos, bearing ] = moveRungeKutta( pos, bearing, speed, turnRate, dt )
    [k1Move, k1Turn] = forwardEuler(bearing, speed, turnRate);
    [k2Move, k2Turn] = forwardEuler(bearing+(dt*k1Turn/2), speed, turnRate);
    [k3Move, k3Turn] = forwardEuler(bearing+(dt*k2Turn/2), speed, turnRate);
    [k4Move, k4Turn] = forwardEuler(bearing+(dt*k3Turn), speed, turnRate);
    
    move = dt/6 * (k1Move + 2*k2Move + 2*k3Move + k4Move);
    pos = pos + move;
    turn = dt/6 * (k1Turn + 2*k2Turn + 2*k3Turn + k4Turn);
    bearing = wrapToPi(bearing + turn);
end