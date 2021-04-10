        
function prac4n10496262(robot)
	% The function needs to start with the following call; 
	%otherwise, your code will not run during automarking.
	robot.powerON();
    
    % For line ax + by + c = 0; where -x -y = 0
    a = -1; b = -1; c =0;
    [x ,y] = robot.getTruePose(); % Get the location of the robot
    
    % Method 1 coords for specific line
    % For line y = -x
%     Nancy = abs( a.*x + b.*y + c);
%     Donkey = sqrt( a.^2 + b.^2);
%     minDistance = Nancy ./ Donkey ;
%     
%     XCoord = (-y + x)/2;
%     YCoord = -XCoord;
    
    % Method 2 coords for general a b c 
    % For line y = -x
    XCoord = -(y - ((b/a)*x)) / ((b/a) + 1) ;
    YCoord = -XCoord;
    
    goal = [XCoord YCoord];
    
    
    
     while(true)
        [x, y ,theta] = robot.getTruePose();
        q = [x y theta];
        vel = ReachPoint(goal , q);
        [lWv, rWv] = VtoWheels(vel(1),vel(2));
        robot.setMotorVel(lWv,rWv)
        pause(0.1)
        
         if sqrt (( goal(2) - q(2) )^2 + (goal(1) - q(1))^2 ) < 0.4
        %if (a * b * c) .* (x * y * 1) / sqrt( (a^2) + (b^2) ) < 0.4
            
            tic
            while(toc < 19.3) % Changing the toc according to Trial and Error
            [x ,y, theta] = robot.getTruePose(); % Getting the location of 
            % the robot at each iteration of the loop
            
            % Controller one 
            Kd = 0.5; % From slide 87
            dLine = (a * b * c) .* (x * y * 1) ./ sqrt( (a^2) + (b^2) );
            SimplePropControllerD = -Kd * dLine;
            
            % Controller two 
            Kh = 1; % From slide 87
            thethaStar = atan2( -a , b );
            SimplePropControllerH = Kh * angdiff(theta, thethaStar);
            
            gammaval = SimplePropControllerH + SimplePropControllerD;
            gammaval = min(gammaval, 0.4);
            gammaval = max(gammaval, -0.4);
            
            KV = 0.246;
            vel = KV * 1;
            
            % Calling VtoWheels
            [lWv, rWv] = VtoWheels(vel ,gammaval);
            % Robot Executes
            robot.setMotorVel(lWv,rWv); 
                    
            end   
    
        % Stop after 20
        robot.setMotorVel(0,0)

            break;
        end
        
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