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