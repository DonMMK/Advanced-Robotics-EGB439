function vel = controlPoint(goal, q)

KV = 0.5;
KH = 0.7; 

EndY = goal(2); EndX = goal(1);
y = q(2); x = q(1); theta = q(3);

theta_goal = atan2(EndY-y, EndX-x);
w=(angdiff(theta,theta_goal))*KH;


if w>0.5
    w=0.5;
elseif w<-0.5
    w=-0.5;
end


e = sqrt((EndX-x)^2 + (EndY-y)^2);
v = KV * e;
v = min(v, 0.07);

vw = [v w];
vel = vw2wheels(vw);

end