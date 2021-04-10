% Make sure to name your function correctly; otherwise, the automatic marking script will not assign you the mark. 
% For example, if your student number is n12345678 and this is a solution for Prac 2, 
% then you should name this function prac2n12345678

% Q) Robot should start driving and tracing the circumference of a circle with a 1 m radius.
% Functions used
% [lWv, rWv] = VtoWheels(V,W)   and robot.SetMotorVel( , )  and [x,y,theta] = robot.getTruePose();
    

function prac2n10496262(robot)
	% The function needs to start with the following call; 
	%otherwise, your code will not run during automarking.
	robot.powerON();
    
    [lWv, rWv] = VtoWheels(0.1,0.1);
     
    % in radians
    robot.setMotorVel(lWv,rWv)
    
	%The function needs to end with the following calls; 
	%otherwise, your code will not run during automarking.
	pause(3)
	robot.powerOFF();
	disp("**********END************");
end    


function [lWv, rWv] = VtoWheels(V,W)

    % Variables
    Radius_Wheel = 0.00975;
    Axle_Length = 0.331;
    %Axle_Length_To_Center = Axle_Length / 2;
    
    % Using equation to calculate each wheel velocity
    lWv = (2* V - W* Axle_Length ) / (2 * Radius_Wheel);
    rWv = (2* V + W* Axle_Length ) / (2 * Radius_Wheel);
    
    
end