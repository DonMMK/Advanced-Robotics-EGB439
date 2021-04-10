% this wrapper function allows the assessment code to access your functions
function out = myFunction(op, varargin)
    switch op
        case 'vel2wheels'
            out = vel2wheels(varargin{:});
        case 'wheels2vel'
            out = wheels2vel(varargin{:});
        case 'qdot'
            out = qdot(varargin{:});
        case 'qupdate'
            out = qupdate(varargin{:});
        case 'control'
            out = control(varargin{:});
    end
end



%----- YOUR CODE GOES BELOW HERE -----

function wheelVel = vel2wheels(vel)
    % Inputs:
    % vel is the velocity vector (v, omega) in units of metres/s and radians/s
    % Return:
    % wheelVel is the wheel velocity vector (vleft, vright) each in the range -100 to +100 to achieve
    % this velocity
end

function vel = wheels2vel(wheelVel)
    % Inputs:
    % wheelVel is the wheel velocity vector (vleft, vright) each in the range -100 to +100
    % Return:
    % vel is the resulting velocity vector (v, omega) in units of metres/s and radians/s
end

function qd = qdot(q, vel)
    % Inputs:
    % q is the configuration vector (x, y, theta) in units of metres and radians
    % vel is the velocity vector (v, omega)
    % Return:
    % qd is the vector (xdot, ydot, thetadot) in units of metres/s and radians/s
end

function qnew = qupdate(q, vel, dt)
    % Inputs:
    % q is the configuration vector (x, y, theta) in units of metres and radians
    % vel is the velocity vector (v, omega)
    % dt is the length of the integration timestep in units of seconds
    % Return:
    % qnew is the new configuration vector vector (x, y, theta) in units of metres and radians at the
    % end of the time interval.
end

function vel = control(q, point)
    % Inputs:
    % q is the initial configuration vector (x, y, theta) in units of metres and radians
    % point is the vector (x, y) specifying the goal point of the robot
end
