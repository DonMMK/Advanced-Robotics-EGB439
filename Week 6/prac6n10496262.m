% Make sure to name your function correctly; otherwise, the automatic marking script will not assign you the mark. 
% For example, if your student number is n12345678 and this is a solution for Prac 2, 
% then you should name this function prac2n12345678

function prac6n10496262(robot)
	% The function needs to start with the following call; 
	%otherwise, your code will not run during automarking.
	robot.powerON();
    

    
    % Use the camera and get the image
     img = robot.getArenaImage();
     %imshow(img)
     pause(0.2);
     
     
     
     
     % Getting the robot vision
     imgFromMask = createMask(img);
     SE = strel('disk' , 10);
     DilatedImg = imdilate(imgFromMask , SE);
     DImg = DilatedImg;
     %idisp(DImg);
     
    pixels = 128; meters = 5;
    SPerMeter = pixels / meters;
    [x ,y] = robot.getTruePose();
    TwoP = 2.5;
    InitPoint = round([x+TwoP , TwoP - y] * SPerMeter);
    PFive = 0.5;
    EndPoint = round([PFive , PFive] * SPerMeter);
    
    % Initialize for find path
    map = DImg;
    start = InitPoint;
    
    goal = EndPoint;
    
    path = findPath(map , start , goal);
    
    path = path/ SPerMeter;
    
    path = [path(:,1)-TwoP , TwoP - path(: , 2)];
    
    speed = 0.5;
    R = 5;
    
    
    % 
        while(true)
            [x ,y ,theta] = robot.getTruePose();
            
            pose = [x , y, theta];
            % PT
            q = pose;
            
            vel = PurePursuit(q, R, speed, path);
            
            
            [lWv, rWv] = VtoWheels(vel(1),vel(2));
            
            robot.setMotorVel(lWv, rWv);

            ConditionA = (x + TwoP) - goal(1) / SPerMeter;
            
            ConditionB = (TwoP - y) - goal(2) / SPerMeter;
            
            BreakCondition = sqrt(abs(ConditionA).^2+ abs(ConditionB).^2);

            %Break condition
            ConstantD = 0.1;
            if (BreakCondition < ConstantD)
                robot.setMotorVel(0,0);
                break;
            end
        end

    
    
   
    
     



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

    KH = 8;
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


function [BW,maskedRGBImage] = createMask(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 26-Apr-2021
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.215;
channel1Max = 0.183;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.000;
channel2Max = 0.181;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.504;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

end


function path = findPath(map, start, goal)
    % Input:
    %   map is an arbitrary size real matrix representing an occupancy grid. The elements are equal to 0 (free space) or 1 (occupied)
    %   start is a 1x2 matrix containing the start coordinate [x y] for the path
    %   goal is a 1x2 matrix containing the end coordinate [x y] for the path
    % Return:
    %   path is an Nx2 matrix containing the ordered path from start to goal, inclusive of end points.  N depends on the map.
    
    dtransform = distanceTransform(map, goal);   % use your own distance transform function
    
    % Get the path 
    path = start;
    
    % Loop until goal is reached
    while true
        
        % Get the 3x3 window around the current coordinates
        GetPathY = path(end, 2); GetPathx = path(end, 1);
        y = GetPathY;
        x = GetPathx;
        
        M = window(dtransform, x, y);
        
        % New Coordinates by using the minval function
        next = minval(M);
        New_YCoord = y + next(2); New_XCoord = x + next(1);
        
        NEW_VAR_Y = New_YCoord;
        NEW_VAR_X = New_XCoord;
         % compute the best path 
        path = [path; NEW_VAR_X, NEW_VAR_Y];
        
        % Check if goal has been reached

        if (dtransform(NEW_VAR_Y, NEW_VAR_X) == 0)
            break;
        end
        
    end
    
end


%----- YOUR CODE GOES BELOW HERE -----
% You are using the manhattan distance measure in Q2.3 when implementing your distance transform. 
% This also means that the grid is only 4-connected for the pathplanning method in the second part of the question.
% Make your own maps to test on, don't just test on the map we provide in grader.


function dtransform = distanceTransform(map, goal)
    % Input:
    %   map is an arbitrary size real matrix representing an occupancy grid. The elements are equal to 0 (free space) or 1 (occupied)
    %   goal is a 1x2 matrix containing the end coordinate [x y] for the path
    % Return:
    %   dtransform is a matrix, the same size as map, whose elements reflect the distance from that cell to the goal
    
    % compute the distance transform of map
    
    [vARlENGTHy , vARlENGTHx] = size(map);
    length_Y = vARlENGTHy;
    length_X = vARlENGTHx;
    
    dtransform = inf(length_Y, length_X);
    dtransform(map == 1)= NaN; % 
    
    % Get the goal as zero
    dtransform( goal(2) , goal(1) ) = 0;
    
    CheckInfinity = inf;
    
    % Use the path planning cost function
    Cost_Func = [ inf, 1, inf;
        1 , 0, 1;
        inf, 1, inf];
    
    % An infinte loop
    CheckInfinity = inf;
    while true
        % Inspect loop
        CurrentBlock = sum(dtransform == inf);
        if ( CurrentBlock >= CheckInfinity)
            break;
        end
        PassBlock = CurrentBlock;
        CheckInfinity = PassBlock;
        
        PassThroughLength_X = length_X;
        PassThroughLength_Y = length_Y;
        for x = 1: PassThroughLength_X
            for y = 1 : PassThroughLength_Y
                
                if isnan(dtransform(y ,x ) )
                    continue;
                end
                
                % Calling the window function from module 2.2
                M = window(dtransform, x, y) ;
                
                % 
                CostFuncPassing = Cost_Func;
                NewMin = CostFuncPassing + M ;
                GettheValue(y,x) = min(min(NewMin));
                dtransform(y, x) = GettheValue(y,x);
                
            end
            
        end
        
    end
    
    
end





function next = minval(M)
% Input:
%  M is a real 3x3 matrix
% Return:
%  next is a 1x2 matrix with elements [x, y] which are the horizontal and vertical coordinates relative to the centre
%       element, of the smallest element in the matrix.
    
        ValueManHat = NaN;
        M(3 , 3) = ValueManHat;
        M(3 , 1) = ValueManHat;
        M(1 , 1) = ValueManHat;
        M(1 , 3) = ValueManHat;
        
        Number_Minimum = min(M , [] , 'all');
        PassMinMin = Number_Minimum;
        
        
        [YPositionValue, XPositionValue] = find(M == PassMinMin);
        
        % Hard code the center values
        CenterMat = M(2,2);
        
        
        PassYPos = YPositionValue;
        PassXPos = XPositionValue;
        
        
        % Center coords
        [CenterY , CenterX] = find(M == CenterMat);
        
        % Rel distance
        PassCenterY = CenterY;
        PassCenterX = CenterX;
        next = [PassXPos - PassCenterX , PassYPos - PassCenterY];
        
        if size(next,1) >= 2
            NextPass = next(2,:);
            next = NextPass; 
        end

        

    
end


function M = window(dtransform, x, y) 
% Input:
%  A an arbitrary sized real matrix, at least 3x3.
%  x the x-coordinate (horizontal index) of the centre of the window.
%  y the y-coordinate (vertical index) of the centre of the window.
% Return:
%  M as 3x3 matrix which are the elements of M centered on the element (x,y).
%
% Should the window extend beyond the edges of the matrix the function must
% return an empty matrix [].
    
    VarA = padarray(dtransform , [1 1], NaN);
    % Using the padarray function inbuilt
    
    % Changes
    subsC = 2;
    Y2 = y + subsC;
    X2 = x + subsC;
    
    PassthroughY = Y2;
    PassthroughX = X2;
    
    % Return the window
    M = VarA( y:PassthroughY , x: PassthroughX);
    
    
    
end



