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