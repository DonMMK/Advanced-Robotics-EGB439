% Make sure to name your function correctly; otherwise, the automatic marking script will not assign you the mark. 
% For example, if your student number is n12345678 and this is a solution for Prac 2, 
% then you should name this function prac2n12345678
% [lWv, rWv] = VtoWheels(V,W)
% robot.SetMotorVel( lWv, rWv )
% [x,y,theta] = robot.getTruePose();

% Move the robot to the required line 
    % Use the angdiff between ThetaStar and Theta to make it parallel
        
    % Turn towards the line  
        
% Line equation ax + by + c = 0;
        
% Follow the line at 40cm to the line
        
% Move for 20 seconds thens stop

% Drive to point & Drive to line in different while loops
        

function prac4n10496262(robot)
	% The function needs to start with the following call; 
	%otherwise, your code will not run during automarking.
	robot.powerON();
    
    % for the line
    a = -1; b = -1; c = 0;

    % get the pose of the robot
    [x y theta] = robot.getTruePose();
    q = [x y theta];
    
    
    
	%The function needs to end with the following calls; 
	%otherwise, your code will not run during automarking.
	pause(3)
	robot.powerOFF();
	disp("**********END************");
end    


% Function for prac4 Takes in the eq of line and converts to heading angle
function  HeadingAngle(a,b )
    Kh = 1; % From slide 87
    thethaStar = atan2d( b ,-a );
    SimplePropControllerH = Kh * angdiff(thetha, thethaStar);
    
end

% x y are the real world coordinates we can obtain from the TruePose
% function
% a b and c are the coeficients in the eq of a line
% Function to drive towards the line 
function DriveTowardtheLine(a , b , c , x , y)
    Kd = 0.5; % From slide 87
    dLine = (a * b * c) .* (x * y * 1) ./ sqrt( (a^2) + (b^2) );
    SimplePropControllerD = -Kd * dLine;
    
end


% Function to calculate the v and w
function FindVandW (q ) % is there a goal
    % Statement
    
end




% % Reach point function from prac 3
% function vel = ReachPoint(goal, q)
% 
% KV = 0.5;
% KH = 0.7; 
% 
% EndY = goal(2); EndX = goal(1);
% y = q(2); x = q(1); theta = q(3);
% 
% theta_goal = atan2(EndY-y, EndX-x);
% w=(angdiff(theta,theta_goal))*KH;
% 
% 
% if w>0.5
%     w=0.5;
% elseif w<-0.5
%     w=-0.5;
% end
% 
% v = KV * sqrt((EndX-x)^2 + (EndY-y)^2);
% v = min(v, 1);
% 
% 
% if sqrt ( (y2 - y1)^2 + (x2 - x1)^2 ) < 0.1 
%     vel = [0 0];
% else 
%     vel = [v w];
% end
% end

% Need to input the v and w! This can be done using another function to
% calc these ref ReachPoint or new func
% VtoWheels function from prac 2
function [lWv, rWv] = VtoWheels(v,w)

    % Variables
    Radius_Wheel = 0.00975;
    Axle_Length = 0.331;
    %Axle_Length_To_Center = Axle_Length / 2;
    
    % Using equation to calculate each wheel velocity
    lWv = (2* v - w* Axle_Length ) / (2 * Radius_Wheel);
    rWv = (2* v + w* Axle_Length ) / (2 * Radius_Wheel);
    
    
end