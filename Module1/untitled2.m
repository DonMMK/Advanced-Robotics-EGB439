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
