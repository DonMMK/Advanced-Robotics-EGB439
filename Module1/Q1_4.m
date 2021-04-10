function vel = control(q, R, speed, path)
    x = q(1); y = q(2);theta = q(3);
    %find value of delta by finding closest distance
    delta =closest_dis(x,y,path);  
    %now find I to use locate the path
    [~, i ] = min(delta);
    locations =looping(find(delta(i+1:end)>=R),path,i) 
    %now build a function to find speed and omega
    vel = find_values(speed,5 * wrapToPi(atan2(locations(2)-q(2),locations(1)-q(1))-theta))
    %now we find the distance
    distance=sqrt((q(1)-path(end,1))^2+(q(2)-path(end,2))^2);
    %determin the stop distance which should be less then 10cm
    stop_distance = 0.01*0.9;
    if distance< stop_distance
         vel(1) = 0; vel(2) = 0;
    end
end

function looping_complete = looping(k,path,i)
value = length(k);
    if value ~= 0
        j = i + k(1) - 1; 
        looping_complete = path(j,:);
    else   
        looping_complete = path(end,:);
    end
end

function find_v_omega = find_values(v,omh)
if v > 0.5
    v = 0.5;
elseif v < -0.5
        v = -0.5;
end
  
if omh > 1
   omh = 1;
elseif omh < -1
        omh = -1;
end
    find_v_omega = [v omh]
end
function closest_dis2=closest_dis(x,y,path)
%this function is how we find delta
 delta = path - [x,y];
 closest_dis2=sqrt(delta(:,1).^2 + delta(:,2).^2);
end

