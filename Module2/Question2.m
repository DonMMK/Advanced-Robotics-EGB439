% this wrapper function allows the assessment code to access your functions
function out = myFunction(op, varargin)
    switch op
        case 'window'
            out = window(varargin{:});
        case 'minwin'
            out = minwin(varargin{:});
        case 'minval'
            out = minval(varargin{:});
    end
end

%----- YOUR CODE GOES BELOW HERE -----

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

function B = minwin(A) 
% Input:
%  A returns a matrix the same size as A where each element of B is the minimum 
% of a 3x3 window of A centered at the corresponding element.  Elements of B 
% that correspond to a window which "falls off the edge" of the matrix A should be set to a value of NaN.
    
    % Makiing an empty matrix
    CornersNew(1,:) = zeros(size(A));
    
    CornersNew(: , end) = NaN;
    CornersNew(: , 1) = NaN;
    CornersNew(end , :) = NaN;
    CornersNew(1 , :) = NaN;
    
    % Getting the size of the window
    WindowSize =  size(A);
    
    Length_Y = WindowSize(2); Length_X = WindowSize(1);
    
    BoundaryY = 2:Length_Y ;BoundaryX = 2:Length_X;
    
    % Using a loop to check
    for Count_I = BoundaryX
        for Count_J = BoundaryY
            CornersNew(Count_J , Count_I) = min(min(window( A, Count_I, Count_J )));
        end
    end
    
    B = CornersNew;
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
