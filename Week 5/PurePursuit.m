function vel = PurePursuit(q, R, speed, path)

% Inputs:
%  q is a 1x3 vector giving the current configuration of the robot in units of metres and radians
%  R is the pure pursuit following distance
%  speed is the forward speed of the vehicle
%  path is an Nx2 matrix containing the path as a set of waypoints, columns are x- and y-coordinates respectively.
% Return:
%  vel is a 1x2 vector containing the requested velocity and turn rate of the robot [v, omega]
    
    % access the pose 
    xValue = q(1);
    yValue = q(2);
    Measuredtheta = q(3);
    Trajectory = path;
    
    Anglechangedsmall = Trajectory - [xValue  , yValue]; 
    HypotenuseCalc = ( Anglechangedsmall(:, 2).^2 + Anglechangedsmall(:, 1).^2 ).^(1/2);
    
    [~ , CounterA] = min(HypotenuseCalc);
    
    Positions = iterating( find(HypotenuseCalc ( CounterA + 1: end) >= R ), path, CounterA);
    % Get the positions through the loop and the speed
    
    RelativeStraightV = speed;

    KH = 6;
    angletheta = atan2( Positions(2) - q(2) , Positions(1) - q(1) );
    AngleMeasured=(angdiff(Measuredtheta,angletheta));
    
    AngleMeasured = wrapToPi(AngleMeasured);
    AngleMeasured = KH*AngleMeasured;
    
    
    if AngleMeasured < -1
            AngleMeasured = -1;
       
    elseif AngleMeasured > 1
            AngleMeasured = 1;  
        
    end 
      
    
    % Getting the vel
    
    vel = [RelativeStraightV AngleMeasured];
    TravelD = ( (q(1) - path(end, 1)).^2 + ( q(2) - path(end, 2) ).^2 ).^(0.5);
    
    TOStopD = 9/1000;
    
    if TravelD < TOStopD 
        % Reset vals
        disp('Distance threshold met')
        vel(1) = 0 ; 
        % Reset vals
        vel(2) = 0 ;
    end
    
    
end



%% Sub function: 
function IterationsToCount = iterating( counterConstant , trajectory , CounterA ) 

     lengthVAR= length(counterConstant);
     conditionforloop =lengthVAR;
    if conditionforloop > 0 || conditionforloop < 0
        staticHolder = CounterA + counterConstant(1) - 1;
        CounterB = staticHolder;
        IterationsToCount = trajectory(CounterB, :);
    else
        IterationsToCount = trajectory(end , : );
    end
    
        
end