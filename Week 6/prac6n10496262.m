% Make sure to name your function correctly; otherwise, the automatic marking script will not assign you the mark. 
% For example, if your student number is n12345678 and this is a solution for Prac 2, 
% then you should name this function prac2n12345678

function prac6n1234567(robot)
	% The function needs to start with the following call; 
	%otherwise, your code will not run during automarking.
	robot.powerON();
    
    % Get the pose of the robot
    [x, y ,theta] = robot.getTruePose();
    pose = [x y theta];
    
    % Use the camera and get the image
     img = robot.getArenaImage();
%     % Define start and goal
%     scalePx = 100 / 2;
%     start = [q(1) q(2)] * scalePx;
%     goal = [-2 2] * scalePx;
%     
    
    
    
    %% J Prac04
    
    % Values
    threshBot = 255 * 0.14;
    threshTop = 255 * 0.45;
    scalePx = 100 / 2;
    
    % Get image and pose
%     img = pb.getLocalizerImage;
%     pose = pb.getLocalizerPose.pose;
    
    % Get 100x100 map from localizer image
    img = imresize(img, [100 100]);
    img = ordfilt2(img, 25, true(5));
    map = img >= threshBot & img <= threshTop;
    
    % Remove obstacles from a rectangle around the robot
    if ~(pose(1) == 0 && pose(2) == 0 && pose(3) == 0)
        x1 = max(round( pose(1) * scalePx - 8), 0);
        x2 = min(round( pose(1) * scalePx + 8), 100);
        y1 = max(round((-pose(2)+2) * scalePx - 8), 0);
        y2 = min(round((-pose(2)+2) * scalePx + 8), 100);
        map(y1:y2,x1:x2) = 0;
    end
    
    % Convert to RGB
    mapRGB = zeros(100, 100, 3);
    mapRGB(:,:,1) = map;
    mapRGB(:,:,2) = ~map;
    
    % Plot map
    figure
    idisp(mapRGB, 'xydata', {[0 2], [2 0]}, 'ynormal')
    hold on
    
    % Plot triangle
    q = [x y theta];
    
    %% J QPLOT
    tri = [0 0 1; -0.15 -0.05 1; -0.15 0.05 1; 0 0 1]';
    x = q(1);
    y = q(2);
    ang = q(3);
    H = [cos(ang) -sin(ang) x; sin(ang) cos(ang) y; 0 0 1];
    Q = H*tri;
    h = plot(Q(1,:), Q(2,:), 'k-');

    
    
    
    
    %%


	%The function needs to end with the following calls; 
	%otherwise, your code will not run during automarking.
	pause(3)
	robot.powerOFF();
	disp("**********END************");
end    


% Calling the working pure pursuit
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



% Sub function: 
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
