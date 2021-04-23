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

