% Make sure to name your function correctly; otherwise, the automatic marking script will not assign you the mark. 
% For example, if your student number is n12345678 and this is a solution for Prac 2, 
% then you should name this function prac2n12345678

function prac6n10454012(robot)
	% The function needs to start with the following call; 
	%otherwise, your code will not run during automarking.
	robot.powerON();
    
    img = robot.getArenaImage();
    pause(0.1)
    
    % changing robot color to path color
    for p = 1:size(img,1)
        for q = 1:size(img,2)
            for r = 1:3
                if img(p,q,r) == 107 || img(p,q,r) == 108 || img(p,q,r) == 106 || img(p,q,r) == 109 || img(p,q,r) == 110
                    img(p,q,r) = 37;
                end
            end
        end
    end
    
    % changing boundry color to obstacle color
    for i = 1:size(img,1)
        for j = 1:size(img,2)
            for k = 1:3
                if img(i,j,k) == 0
                    img(i,j,k) = 188;
                end
            end
        end
    end
    
    % vision
    imgg = createMask(img);
    se = strel('disk', 9);
    imggg = imdilate(imgg, se);
    map = imggg;
    idisp(map);
    
    scale = 128/5; % pixels per meter
    [x, y] = robot.getTruePose(); start = round([x+2.5, 2.5-y] * scale);
    goal = round([0.5, 0.5] * scale);
    path = findPath(map, start, goal);
    
    path = path/scale;
    path = [path(:,1)-2.5, 2.5-path(:,2)];
    
    R = 0.4;
    speed = 0.8;
    
    while (true)
%         [~, ~, theta] = robot.getTruePose();
%         
%         if ~(abs(theta) < deg2rad(10))
%             
%             % heading error to goal
%             goal_theta = 0;
%             pos_theta = theta;
%             heading_error = goal_theta - pos_theta;
%             
%             % wrapping between [-180 180]
%             heading = wrapToPi(heading_error);
%             
%             % controlling and clipping values
%             Kh = 3;
%             W = Kh * heading;
%             W = min(W, 1.3);
%             W = max(W, -1.3);
%             
%             V = 0;
%             
%             [lWv, rWv] = VtoWheels(V, W);
%             robot.setMotorVel(lWv, rWv);
%             pause(0.1)
%             
%         else
            
            while (true)
                [x, y, theta] = robot.getTruePose();
                q = [x, y, theta];
                vel = control(q, R, speed, path);
                [lWv, rWv] = VtoWheels(vel(1), vel(2));
                robot.setMotorVel(lWv, rWv);
                cond = hypot((x+2.5)-goal(1)/scale,(2.5-y)-goal(2)/scale);
                
                if (cond < 10/100)
                    robot.setMotorVel(0, 0);
                    break;
                end
            end
        % end
    end


	%The function needs to end with the following calls; 
	%otherwise, your code will not run during automarking.
	pause(3)
	robot.powerOFF();
	disp("**********END************");
end
