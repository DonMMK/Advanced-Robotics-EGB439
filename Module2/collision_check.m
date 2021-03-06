function collide = collision_check(q)
    % q is a 1x3 vector containing the robot configuration with elements x, y and theta (in degrees)
    % collide is true if the robot intersects any of the obstacles, otherwise it is false
    
    close all;
    % Hard coding the obstacle positions
    TheLongOne = [5 ,6 ;
                  5 ,10;
                  6 ,10;
                  6 ,6];
              
              
    BottomOne = [5 ,0 ;
                  5 ,4;
                  6 ,4;
                  6 ,0];
              
    Smallone = [2 ,5 ;
                  2 ,7;
                  3 ,7;
                  3 ,5];
              
    LongTopObs = TheLongOne;
    LongBotObs = BottomOne;
    ShortBTObs = Smallone;
    
    Obstacles = [ polyshape(LongTopObs) ,
        polyshape(LongBotObs) ,
        polyshape(ShortBTObs) ];
    hold on
    plot(Obstacles, 'FaceColor', 'r')
              
    %
    % Declaring the robot
    Length = 3;
    Width = 1.5;
    
%     RotationalMat = [ cos(q(3)) -sin(q(3));
%                       sin(q(3)) cos(q(3))];
     WV = Width/2;
     LV = Length/2;
                  
    RobotS = [ -LV , -WV;
               -LV ,  WV;
                LV ,  WV;
                LV , -WV;
                        ] + [q(1) q(2)];
                    
                    
    % Robot
    
    
    
    % Robot rotation matrix
    shifted = [LV ,WV];
    PointAboutTurn = [RobotS(1, :) + shifted];
    PATDum = PointAboutTurn;
    MoveBRobot = rotate(polyshape(RobotS) , q(3) , PATDum);
    
    % to plot
    plot(MoveBRobot , 'FaceColor' ,'b')
    hold off
    
    count = 0;
    onee = 1;
    Final = length(Obstacles);
    
    for loop = 1 : Final
        
        VarCollide = intersect(Obstacles(loop) , MoveBRobot);
        
        if VarCollide.NumRegions > 0
            count = count + onee;
        end
        
    end
    
    % condition
    
    if count <= 0
        collide = false;
        
    else
        collide = true;
    end
    
    
    
    
end