% In Q2.1, the focus of the question is on identifying which type of vehicle you have 
% (2a,2b,3a, or 3b from the lecture slides) and then simply implementing the relationship 
% Don't use a for loop in Q2.1 and iterate for a set amount of time. 
% Think about the stopping condition for the vehicle and implement that in code.

function myFunction(q0, sensorFunction)
    
    q = q0;
    %sensors = sensorFunction(q);
    InitVar = 0;
    sensors(1) = InitVar;
    sensors(2) = InitVar;
    path=[q(1) q(2)]; 
    %sensors = sensorFunction(q0)
  
    %SensorL = InitVar;
    %SensorR = InitVar;
    
    
    while(sensors(1) < 0.99 && sensors(2) < 0.99)
        
        sensors = sensorFunction(q);
        
        vel = [1-sensors(1) , 1-sensors(2)]; 
        % compute wheel speeds
        q = qupdate(q, vel); 
        x=q(1);
        y=q(2);
        
        % store all values in path
        path=[path; x, y];
        
    end
    plot(path(:,1),path(:,2))
end
    


