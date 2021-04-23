function out = Question3(op, varargin)
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
    
    [length_Y , length_X] = size(map);
    dtransform( map == 1 ) = Nan;
    
    dtransform( goalCell(2) , goalCell(1) ) = 0;
    
    % Use the path planning cost function
    Cost_Func = [ 2^0.5, 1, 2^0.5; 1 , 0, 1; 2^0.5, 1, 2^0.5];
    
    % An infinte loop
    CheckInfinity = inf;
    while true
        CurrentBlock = sum (dtransform == inf);
        if ( CurrentBlock >= CheckInfinity)
            break
        end
        CheckInfinity = CurrentBlock;
        
        CountX = 1; CountY = 1;
        while CountX < length_X
            while CountY < length_Y
                
                if isnan( dtransform(CountY ,CountX ) )
                    continue;
                end
                
                % Calling the window function from module 2.2
                M = window(A, x, y) ;
                
                % 
                NewMin = Cost_Func + M ;
                dtransform(y, x) = min(min(NewMin));
                
                
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
        y = path(end, 2);x = path(end, 1);
        M = window(dtransform, x, y);
        
        % New Coordinates by using the minval function
        next = minval(M);
        New_YCoord = y + next(2); New_XCoord = x + next(1);
         % compute the best path 
        path = [path; New_XCoord New_YCoord];
        
        % Check if goal has been reached
        if (dtransform(New_YCoord, New_XCoord) == 0)
            break;
        end
        
    end
    
end