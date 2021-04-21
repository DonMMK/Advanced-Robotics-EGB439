% In Q2.1, the focus of the question is on identifying which type of vehicle you have 
% (2a,2b,3a, or 3b from the lecture slides) and then simply implementing the relationship 
% Don't use a for loop in Q2.1 and iterate for a set amount of time. 
% Think about the stopping condition for the vehicle and implement that in code.

function myFunction(q0, sensorFunction)
    % q0 is a 3x1 vector giving the initial configuration of the robot in units of metres and degrees
    % sensorFunction is a function that accepts the robot configuration and 
    % returns a vector containing the left and right sensor values
    % eg. sensors = sensorFunction( q )
    % where sensors is a 2x1 vector containing the left and right sensor readings
    q = q0;
    %sensors = sensorFunction(q);
    sensors(1) = 0.5;
    sensors(2) = 0.5;
    path=[q(1) q(2)];
    while(sensors(1) < 0.99 && sensors(2) < 0.99)
        sensors = sensorFunction(q)
        vel = [1-sensors(1) , 1-sensors(2)]; 
        % compute wheel speeds
        q = qupdate(q, vel); 
        x=q(1);y=q(2);
        
        % store all values in path
        path=[path; [x, y]];
        
    end
    plot(path(1,:),path(2,:))
end
    