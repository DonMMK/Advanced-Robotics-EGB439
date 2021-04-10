close all; clear; clc

sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)

sim.simxFinish(-1); % just in case, close all opened connections

clientID = sim.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

if (clientID > -1)
    
    disp('Connected to remote API server');
    
    % Retrieves an object handle using the name of the Motor.
    % [returnCode, handle] = sim.simxGetObjectHandle(clientId, objectName, operationMode)
    [returnCodel, lm_handle] = sim.simxGetObjectHandle(clientID, 'lMotor', sim.simx_opmode_blocking); 
    [returnCoder, rm_handle] = sim.simxGetObjectHandle(clientID, 'rMotor', sim.simx_opmode_blocking);
    
    
    sim.simxSetJointTargetVelocity(clientID, lm_handle, 57.6*pi/180, sim.simx_opmode_oneshot);
    sim.simxSetJointTargetVelocity(clientID, rm_handle, 57.6*pi/180, sim.simx_opmode_oneshot);
    
    pause(1);
    
    sim.simxSetJointTargetVelocity(clientID, lm_handle, 0, sim.simx_opmode_oneshot);
    sim.simxSetJointTargetVelocity(clientID, rm_handle, 0, sim.simx_opmode_oneshot);
    

else
    
    disp('Failed connecting to remote API server');
    
end

sim.delete(); % call the destructor!