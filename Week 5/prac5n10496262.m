% Make sure to name your function correctly; otherwise, the automatic marking script will not assign you the mark. 
% For example, if your student number is n12345678 and this is a solution for Prac 2, 
% then you should name this function prac2n12345678

% Use linspasce for the plot and use drive to point for number of points
function prac5n10496262(robot)
	% The function needs to start with the following call; 
	%otherwise, your code will not run during automarking.
	robot.powerON();
    
    

    
    
    % Initialise
    t = linspace(0 , 2 * pi , 100);
    %A = 0.5 / sqrt(2); % Value chosen such that x is limited to -0.5m and 0.5m

    % Equations given
    X_Function = (1.8 .* cos(t) ) ./ ( 1 + (sin(t).^2) );
    Y_Function = (3 .* sin(t) .* cos(t)) ./ (1 + (sin(t).^2) );

    % Plotting the graph
    figure(1);
    hold on
    grid on
    title('Leminscate of Bernoulli')
    xlabel('x(t)')
    ylabel('t')
    plot( X_Function , Y_Function , 'k')

    % Convert the x and y equations into a path
    for i = 1:length(t)
        X_Function(i)
        Y_Function(i) 

    end
    path = [X_Function' ,Y_Function'];

    
        
    % Robot motion
    Nsteps = 100000;
    for step = 1: Nsteps
        [x, y ,theta] = robot.getTruePose();
        q = [x y theta];
        R = 0.6;
        speed = 0.7;
        
%         % Drive point to get to the P1
%         ReachFirstPoint = false;
%         while ReachFirstPoint == false
% 
%             goal = [1.8 0];
%             vel = ReachPoint(goal , q);
%             [lWv, rWv] = VtoWheels(vel(1),vel(2)); % << Calling v to wheels
%             robot.setMotorVel(lWv,rWv)
%             
%             % checking if the robot reaches the first point
%             [x, y ,theta] = robot.getTruePose();
%             q = [x y theta];
%             if [q(1) q(2)] == goal
%                 ReachFirstPoint = true;
%             end
%         end
        
        
        % Pure pursuit according to the leminscate of bernoulli
        vel = PurePursuit(q, R, speed, path); 
        [lWv, rWv] = VtoWheels(vel(1),vel(2)); % << Calling v to wheels
        robot.setMotorVel(lWv,rWv)
        
        % if the robot is less than 0.1m of the goal
        %if sqrt (( goal(2) - q(2) )^2 + (goal(1) - q(1))^2 ) < 0.1
        %    pause(1);
        %    robot.setMotorVel(0,0)
        %    break;
        %end
        
    end
    
	%The function needs to end with the following calls; 
	%otherwise, your code will not run during automarking.
	pause(3)
	robot.powerOFF();
	disp("**********END************");
end    




% Moslty unchanged code Reach point function from prac 3
function vel = ReachPoint(goal, q)

    KV = 0.45;
    KH = 1; 

    EndY = goal(2); EndX = goal(1);
    y = q(2); x = q(1); theta = q(3);

    theta_goal = atan2(EndY-y, EndX-x);
    w=(angdiff(theta,theta_goal))*KH;


    if w>0.5
        w=0.5;
    elseif w<-0.5
        w=-0.5;
    end

    v = KV * sqrt((EndX-x)^2 + (EndY-y)^2);
    v = min(v, 0.4);

    % if sqrt ( (y2 - y1)^2 + (x2 - x1)^2 ) < 0.4 
    %     vel = [0 0];
    % else 
        vel = [v w];
    % end
end



% Moslty unchanged code Reach point function from prac 2
function [lWv, rWv] = VtoWheels(v,w)

    % Variables
    Radius_Wheel = 0.0975;
    Axle_Length = 0.331;
    %Axle_Length_To_Center = Axle_Length / 2;
    
    % Using equation to calculate each wheel velocity
    lWv = (2* v - w* Axle_Length ) / (2 * Radius_Wheel);
    rWv = (2* v + w* Axle_Length ) / (2 * Radius_Wheel);
    
    
end
