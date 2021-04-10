% Write the configuration of the vehicle (1x3 vector) in units of metres and degrees.  
Q = [1.3, 2.1, 40]


% Write a 3x3 homogeneous transformation matrix that describes the pose of this robot 
% with respect to the world coordinate frame.
TR = [  cosd(40) , -sind(40) , 1.3; sind(40) , cosd(40) , 2.1;  0 , 0 , 1  ]


% Write a 3x3 homogeneous transformation matrix that describes the pose of the sensor 
% with respect to the world coordinate frame.
TS = [  cosd(40) , -sind(40) , 1.3; sind(40) , cosd(40) , 2.1;  0 , 0 , 1  ] *[  cosd(-8) , -sind(-8) , 0.4; sind(-8) , cosd(-8) , -0.15;  0 , 0 , 1  ]


% Write the position of the navigation target relative to the sensor in polar 
% coordinate form as a 2x1 vector  (in units of metres and degrees, respectively)
StoreArray = [-1.2 ; 6.6 ; 1]
SensorToLoc = inv(TS) * StoreArray ; 
longdistance = ( SensorToLoc(1)^2  + SensorToLoc(2)^2 )
angle = atan2d( SensorToLoc(2) , SensorToLoc(1) ); 
PP = [longdistance;angle]