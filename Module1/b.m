%% Matlab Grader Question 1.4

% purepursuit = load('purepursuit.mat'); 
% path = purepursuit.robot_traj;
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
    vel = control(q, R, speed, path)
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
    T = transMatrix(x, y, theta);
    pp = T*points;
    % clever plot stuff
    plot(pp(1,:), pp(2,:))
end

function vel = control(q, R, speed, path)
% Inputs:
%  q is a 1x3 vector giving the current configuration of the robot in units of metres and radians
%  R is the pure pursuit following distance
%  speed is the forward speed of the vehicle
%  path is an Nx2 matrix containing the path as a set of waypoints, columns are x- and y-coordinates respectively.
% Return:
%  vel is a 1x2 vector containing the requested velocity and turn rate of the robot [v, omega]

% Initialization
path_x=path(:,1);path_y=path(:,2);
qx=q(1); qy=q(2);
counter=0;

% first find if the car is in the path:
Lia=ismember(path,[qx qy],'row');
counter=find(Lia==1);

% find the closest distance:
distances = sqrt((path_x-qx).^ 2 + (path_y-qy).^ 2);
[~, location] = min(distances);
final_x = path_x(location);
final_y = path_y(location);


% find the closest point within 0.3m radius:
D=sqrt((path_x-final_x).^2+(path_y-final_y).^2)-R;
filter=find((D<=R));

if(counter~=0)
    idx=find(filter<counter);
    filter(idx)=[];
end

% find the location
DIFF=diff(filter);

% find the location
if all(DIFF==1)==1
    L = filter(end);
    counter=L;
elseif all(DIFF==1)==0
    L = find(DIFF>1);
    L=filter(L);
    counter=L;
end
final_x=path_x(L);
final_y=path_y(L);


xg=final_x;
yg=final_y;


% vel
x=q(1); y=q(2); theta=q(3);
theta_g = atan2((yg-y),(xg-x));

gamma = angdiff(theta,(theta_g));
w=gamma;

%condtion for clipping speed
if speed<-0.5
    speed=-0.5;
elseif speed>0.5
    speed=0.5;
end
%condtion for clipping W
if w<-1
    w=-1;
elseif w>1
    w=1;
end

v=speed;
vel=[v,w];

end