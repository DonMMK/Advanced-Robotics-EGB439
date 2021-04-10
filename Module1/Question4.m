close all; clear; clc

path = [linspace(0, 1, 50)' linspace(0, 2, 50)';...
    linspace(1, 2, 50)' linspace(2, 0, 50)';...
    linspace(2, 1, 50)' linspace(0, -2, 50)';...
    linspace(1, 0, 50)' linspace(-2, 0, 50)']; 
q = [0.2 0 0];
R = 0.3;
speed = 0.5;
dt = 0.15;

figure(1);
qplot(q);hold on;
plot(path(:,1), path(:,2));
for step = 1:150
    vel = controlll(q, R, speed, path)
    q = qupdate(q, vel, dt);
    qplot(q);
end


%% Functions 

function qnew = qupdate(q, vel, dt)
    % Inputs:
    % q is the configuration vector (x, y, theta) in units of metres and radians
    % vel is the velocity vector (v, omega)
    % dt is the length of the integration timestep in units of seconds
    % Return:
    % qnew is the new configuration vector vector (x, y, theta) in units of metres and radians at the
    % end of the time interval.
    V = vel(1);
    omega = vel(2);
    theta = q(3);
    xdot = V*cos(theta);
    ydot = V*sin(theta);
    thetadot = omega;
    
    qdot = [xdot, ydot, thetadot];
    qstep = qdot*dt;
    qnew = q+qstep;
end

function qplot(q)
% use plot command to draw the triangle as shown above
    % (x,y) position of the robot on the xy-plane
    % theta, heading angle of the robot in radians
    x = q(1);
    y = q(2);
    theta = q(3);
    xp = [0, -0.2, -0.2, 0];
    yp = [0, -0.075, 0.075, 0];
    points = [xp; yp; 1 1 1 1];
    
    % Transformation Matrix
    T = [ cosd(theta) -sind(theta) x;
       sind(theta)  cosd(theta) y;
                0           0    1 ];
    pp = T*points;
    % clever plot stuff
    plot(pp(1,:), pp(2,:))
end


function vel = controlll(q, R, speed, path)

% Inputs:
%  q is a 1x3 vector giving the current configuration of the robot in units of metres and radians
%  R is the pure pursuit following distance
%  speed is the forward speed of the vehicle
%  path is an Nx2 matrix containing the path as a set of waypoints, columns are x- and y-coordinates respectively.
% Return:
%  vel is a 1x2 vector containing the requested velocity and turn rate of the robot [v, omega]

    % clever stuff
    
    % access the pose 
    xValue = q(1);
    yValue = q(2);
    Measuredtheta = q(3);
    Trajectory = path;
    
    Anglechangedsmall = Trajectory - [xValue  , yValue]; 
    HypotenuseCalc = ( Anglechangedsmall(:, 2).^2 + Anglechangedsmall(:, 1).^2 ).^(1/2);
    
    [~ , CounterA] = min(HypotenuseCalc);
    
    Positions = iterating( find(HypotenuseCalc ( CounterA + 1: end) >= R ), path, CounterA);
    % Get the positions through the loop and the speed
    
    RelativeStraightV = speed;

    KH = 5;
    angletheta = atan2( Positions(2) - q(2) , Positions(1) - q(1) );
    AngleMeasured=(angdiff(Measuredtheta,angletheta));
    
    AngleMeasured = wrapToPi(AngleMeasured);
    
    % Get the angles % Note to self the other redundant code was removed
    
    AngleMeasured = KH*AngleMeasured;
    
    
    if AngleMeasured < -1
            AngleMeasured = -1;
       
    elseif AngleMeasured > 1
            AngleMeasured = 1;  
        
    end 
    
    
    
    % Getting the vel
    
    vel = [RelativeStraightV AngleMeasured];
    
    
    TravelD = ( (q(1) - path(end, 1).^2 + ( q(2) - path(end, 2) ).^2 ) ).^(0.5);
    
    TOStopD = 9/1000;
    
    if TravelD < TOStopD 
        % Reset vals
        vel = [0 0] ; 
        % Reset vals
        %vel(2) = 0 ;
    end
    
    
end



%% Sub function: 
function IterationsToCount = iterating( counterConstant , trajectory , CounterA ) 

     lengthVAR= length(counterConstant);
     conditionforloop =lengthVAR;
    if conditionforloop > 0 || conditionforloop < 0
        staticHolder = CounterA + counterConstant(1) - 1;
        CounterB = staticHolder;
        IterationsToCount = trajectory(CounterB, :);
    else
        IterationsToCount = trajectory(end , : );
    end
    
        
end