function out = myFunction(op, varargin)
    switch op
        case 'dxform'
            out = distanceTransform(varargin{:});
        case 'findpath'
            out = findPath(varargin{:});
    end
end


%     % Get the minimum rows using the min function
%     [VarN, VarA] = min(M);
%     VarPassN = VarN;
%     VarPassA = VarA;
%     
%     % Min function to get the column
%     [~ , VarB] = min(VarPassN);
%     VarPassB = VarB;
% 
%     
%     % Get the required values for the x and y
%     shifting = 2;
%     y = VarPassB - shifting;
%     x = VarPassA(VarPassB) - shifting;
%     
%     
%     % Return the elements x and y in the next matrix
%     next = [y x];
%         
