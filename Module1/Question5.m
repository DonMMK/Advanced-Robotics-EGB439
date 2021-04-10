% Initialise the car
velocity = -0.7;
angularvelocity = ( velocity/3 )*tand(15);
init_pose = [0 0 0];
change_pose = 0.05;
velocity_array = [velocity angularvelocity];


% Get the poses

    PoseArr = [];
    for looping = 1:100 
        init_pose = ChangePose(init_pose, velocity_array , change_pose);
        PoseArr(end + 1, :) = [init_pose];           
       
    end
        
XValuesCar =  PoseArr(: , 1);
YValuesCar =  PoseArr(: , 2);

angle = PoseArr(: , 3);

XValuesCar = [0  XValuesCar'];
YValuesCar = [0  YValuesCar'];
angle = [0 angle'];
    
% Homogenous matrix 

ValuesXForCar = [3.5 3.5 -0.5 -0.5 3.5];
ValuesYForCar = [0.85 -0.85 -0.85 0.85 0.85];
ExactCoord = [ValuesXForCar; ValuesYForCar ; 1 1 1 1 1];

XValuesArray = zeros(101 , 5);
YValuesArray = zeros(101 , 5);

    for count = 1: 101
        
        MatrixTrans = [ cos(angle(count)) -sin(angle(count)) XValuesCar(count)  ; sin(angle(count)) cos(angle(count)) YValuesCar(count) ; 0 0 1];

        FinalAnswerforArray = MatrixTrans * ExactCoord;
        
        ValuesXForCar( count , 1) = (FinalAnswerforArray(1 ,1));
        ValuesXForCar( count , 2) = (FinalAnswerforArray(1 ,2));
        ValuesXForCar( count , 3) = (FinalAnswerforArray(1 ,3));
        ValuesXForCar( count , 4) = (FinalAnswerforArray(1 ,4));
        ValuesXForCar( count , 5) = (FinalAnswerforArray(1 ,5));
        
        ValuesYForCar( count , 1) = (FinalAnswerforArray(1 ,1));
        ValuesYForCar( count , 2) = (FinalAnswerforArray(1 ,2));
        ValuesYForCar( count , 3) = (FinalAnswerforArray(1 ,3));
        ValuesYForCar( count , 4) = (FinalAnswerforArray(1 ,4));
        ValuesYForCar( count , 5) = (FinalAnswerforArray(1 ,5));
        
    end
    
% To plot the graph

    figure(1)
    plot( ValuesXForCar(: , 3) , ValuesYForCar(: , 3) , 'Tag' , 'Right' )
    hold on
    plot(ValuesXForCar(:, 4) , ValuesYForCar(:, 4), 'Tag' , 'Left')
    
    plot( [ ValuesXForCar(21 , 3) , ValuesXForCar(21 , 4) ] , [ ValuesYForCar(21 , 3) , ValuesYForCar(21 , 4) ] , 'Tag' , '1Second' )
    
    plot( ValuesXForCar(1 ,:) , ValuesYForCar(1 , :) , 'Tag' , 'Car')
        
% plot(  ...., 'Tag', 'Car')
% plot(  ...., 'Tag', 'Left')
% plot(  ...., 'Tag', 'Right')
% plot(  ...., 'Tag', '1Second')

function PoseNEW = ChangePose(init_pose, velocity_array , change_pose)

XValuesCar = init_pose(1);
YValuesCar = init_pose(2);
angle = init_pose(3);


AngledX = velocity_array(1) * cos(angle);
AngledY = velocity_array(1) * sin(angle);
Angledd = velocity_array(2);

PoseNEW = [AngledX AngledY Angledd] * change_pose;
PoseNEW = PoseNEW + init_pose;

end

