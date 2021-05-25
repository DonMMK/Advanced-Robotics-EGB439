%load odomTestData.mat

num_steps = length(odomTestData);
odom_traj = zeros(3,num_steps);

for t=1:num_steps
    if (t == 1)
        old_ticks = odomTestData(t).encoders;
        odom_pose = [0 0 0]';        
    else
        % odomTestData(t).encoders is [left_ticks right_ticks]
        new_ticks = odomTestData(t).encoders;
        % complete the function get_odom below. 
        [d , dth] = get_odom(new_ticks,old_ticks);        
        % complete the function move below.
        odom_pose = move(odom_pose,d,dth);
        old_ticks = new_ticks;
    end
    odom_traj(:,t) = odom_pose;
end


function [d , dth] = get_odom(new_ticks,old_ticks)
%inputs: new_ticks , old_ticks both are 1x2 vectors [left_ticks right_ticks]
%outputs: d  distance traveled in meters
%        dt angle rotated in radians 

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
%inputs: current_pose is a 3x1 vector [x; y; theta] of the robot (theta in radians)
%        d  distance traveled in meters
%        dt angle rotated in radians 
%outputs: next_pose is a 3x1 vector [x; y; theta] of the robot (theta in radians)

    X_dash = D_central 
    Y_dash = 


end


