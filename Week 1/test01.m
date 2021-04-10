sim.simxFinish(-1); % just in case, close all opened connections

clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if (clientID > -1)
    
    disp('Connected to remote API server');
    
    % Retrieves an object handle using the name of the Motor.
    % [returnCode, handle] = sim.simxGetObjectHandle(clientId, objectName, operationMode)
    [returnCode, lm_handle] = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking); 
    
    %
    %
    sim.simxSetJointTargetVelocity(clientID, lm_handle, 10*pi/180, sim.simx_opmode_oneshot);
    
    pause(5);
    
    sim.simxSetJointTargetVelocity(clientID, lm_handle, 0, sim.simx_opmode_oneshot);
    

else
    
    disp('Failed connecting to remote API server');
    
end

sim.delete(); % call the destructor!