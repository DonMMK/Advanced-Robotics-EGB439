
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







