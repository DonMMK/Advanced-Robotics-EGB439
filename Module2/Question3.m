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
    dtransform( map == 1 ) = NaN;
    
    dtransform( goal(2) , goal(1) ) = 0;
    
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




function M = window(A, x, y) 
% Input:
%  A an arbitrary sized real matrix, at least 3x3.
%  x the x-coordinate (horizontal index) of the centre of the window.
%  y the y-coordinate (vertical index) of the centre of the window.
% Return:
%  M as 3x3 matrix which are the elements of M centered on the element (x,y).
%
% Should the window extend beyond the edges of the matrix the function must
% return an empty matrix [].
    
%     % Using the padarray function;
%     VarA = padarray(A , [1 1], NaN);
%     
%     % Get Parameters
%     Y2 = y + 2;
%     X2 = x + 2;
%     
%     % Return the window
%     M = VarA( y:Y2 , x: X2);

    SizeofA = size(A);
    Length_Y = SizeofA(1); Length_X = SizeofA(2);
    
    Y_Across = y+1; Y_Down = y-1;
    X_Across = x+1; X_Down = x-1; 
    
    MinVal = 2;
    
    Subs = 1;
    ymax = Length_Y- Subs; xmax = Length_X- Subs;
    
    
    isOutOfBounds = x < MinVal || y < MinVal || x > xmax || y > ymax;
    if (isOutOfBounds)
        M = [];
    else
        M = A(Y_Down:Y_Across,X_Down:X_Across);
    end
end


function next = minval(M)
% Input:
%  M is a real 3x3 matrix
% Return:
%  next is a 1x2 matrix with elements [x, y] which are the horizontal and vertical coordinates relative to the centre
%       element, of the smallest element in the matrix.
    
%     % Get the minimum rows using the min function
%     [VarN] = min(M);
%     
%     
%     % Min function to get the column
%     [~ , VarB] = min(VarN);
%     [~,VarA] = min(M);
% 
%     
%     % Get the required values for the x and y
%     shifting = 2;
%     y = VarB - shifting;
%     x = VarA(VarB) - shifting;
%     
%     
%     % Return the elements x and y in the next matrix
%     next = [x y];
    
    % Gives the minumum element among rows/ columns
    [PosA , PosB] = find (M == min( min(M)) );
    
    
    % Hard coding the center value
      RelPath = [2 - PosB,2 - PosA];
    
      next = [RelPath(1) RelPath(2) ];
    
    
    
    

    
end