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
    x = q(1); y = q(2);theta = q(3);
    %find value of delta by finding closest distance
    delta =closest_dis(x,y,path);  
    %now find I to use locate the path
    [~, i ] = min(delta);
    locations =looping(find(delta(i+1:end)>=R),path,i) 
    %now build a function to find speed and omega
    vel = find_values(speed,5 * wrapToPi(atan2(locations(2)-q(2),locations(1)-q(1))-theta))
    %now we find the distance
    distance=sqrt((q(1)-path(end,1))^2+(q(2)-path(end,2))^2);
    %determin the stop distance which should be less then 10cm
    stop_distance = 0.01*0.9;
    if distance< stop_distance
         vel(1) = 0; vel(2) = 0;
    end
end

function looping_complete = looping(k,path,i)
value = length(k);
    if value ~= 0
        j = i + k(1) - 1; 
        looping_complete = path(j,:);
    else   
        looping_complete = path(end,:);
    end
end

function find_v_omega = find_values(v,omh)
if v > 0.5
    v = 0.5;
elseif v < -0.5
        v = -0.5;
end
  
if omh > 1
   omh = 1;
elseif omh < -1
        omh = -1;
end
    find_v_omega = [v omh]
end
function closest_dis2=closest_dis(x,y,path)
%this function is how we find delta
 delta = path - [x,y];
 closest_dis2=sqrt(delta(:,1).^2 + delta(:,2).^2);
end

