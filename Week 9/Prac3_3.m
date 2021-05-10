load map1r1.mat 

num_steps = length(data);
odom_traj = zeros(3,num_steps);

for t=1:num_steps
    if (t == 1)
        old_ticks = data(t).encoders;
        odom_pose = [0 0 0]';        
    else
        new_ticks = data(t).encoders;
        [d , dth] = get_odom(new_ticks,old_ticks);        
        odom_pose = move(odom_pose,d,dth);
        old_ticks = new_ticks;
    end
    odom_traj(:,t) = odom_pose;
end


% NOT REQUIRED IN GRADER
for t=1:num_steps
    plot_robot(odom_traj(:,t));
    pause(0.1);
end

function [d , dth] = get_odom(new_ticks,old_ticks)

    Num_tick_per_rot = 370;
    Wheel_diameter = 0.065;
    Wheel_axis = 0.15;
    
    delta_tick_left = new_ticks(:,1) - old_ticks(:,1);
    delta_tick_right = new_ticks(:,2) - old_ticks(:,2);
    
    D_left = (2 * pi * 0.5 * Wheel_diameter * delta_tick_left) / Num_tick_per_rot;
    D_right= (2 * pi * 0.5 * Wheel_diameter * delta_tick_right) / Num_tick_per_rot;
    
    d = ( D_left + D_right ) / 2;
    dth =  (D_right- D_left)/ Wheel_axis;



end

function next_pose = move(current_pose, d , dth)
    x_dash = current_pose(1,:) + d * cos(current_pose(3,:));
    y_dash = current_pose(2,:) + d * sin(current_pose(3,:));
    theta_dash = current_pose(3,:) + dth;
    next_pose = [x_dash; y_dash; theta_dash];

end

% NOT REQUIRED IN GRADER     
function plot_robot(r)
    

end