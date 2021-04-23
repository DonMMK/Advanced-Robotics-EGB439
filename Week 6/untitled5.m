%% After return the window line 140

%     SizeofA = size(dtransform);
%     Length_Y = SizeofA(1); Length_X = SizeofA(2);
%     
%     Y_Across = CountY+1; Y_Down = CountY-1;
%     X_Across = CountX+1; X_Down = CountX-1; 
%     
%     MinVal = 2;
%     
%     Subs = 1;
%     ymax = Length_Y- Subs; xmax = Length_X - Subs;
%     
%     
%     isOutOfBounds = CountX < MinVal || CountY < MinVal || CountX > xmax || CountY > ymax;
%     if (isOutOfBounds)
%         M = [];
%     else
%         M = dtransform(Y_Down:Y_Across,X_Down:X_Across);
%     end


%%