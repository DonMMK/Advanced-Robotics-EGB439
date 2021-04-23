function out = myFunction(op, varargin)
    switch op
        case 'dxform'
            out = distanceTransform(varargin{:});
        case 'findpath'
            out = findPath(varargin{:});
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


function next = minval(M)
% Input:
%  M is a real 3x3 matrix
% Return:
%  next is a 1x2 matrix with elements [x, y] which are the horizontal and vertical coordinates relative to the centre
%       element, of the smallest element in the matrix.
    
    % Get the minimum rows using the min function
    [VarN, VarA] = min(M);
    VarPassN = VarN;
    VarPassA = VarA;
    
    % Min function to get the column
    [~ , VarB] = min(VarPassN);
    VarPassB = VarB;

    
    % Get the required values for the x and y
    shifting = 2;
    y = VarPassB - shifting;
    x = VarPassA(VarPassB) - shifting;
    
    
    % Return the elements x and y in the next matrix
    next = [y x];
        

    
end