% The maximum steering angle in degrees
% Initialise varibles
WBLength = 2.5;
BWRadius = 6;
MS = atan2d(WBLength, BWRadius );

% the radius of the arc of the reference point of the car in metres
% Initialise varibles
NSWangle = 12;
R = WBLength / tand(NSWangle);


% coordinate of the ICR in meters as a 2x1 vector
WorldCoordinate = [20 30 30];
ToRadius = [ cosd(WorldCoordinate(3)) -sind(WorldCoordinate(3)) WorldCoordinate(1); sind(WorldCoordinate(3)) cosd(WorldCoordinate(3)) WorldCoordinate(2); 0 0 1];

RobotFrame = [0; -R;1];

ArrayforICR = ToRadius * RobotFrame;
ICR = [ArrayforICR(1) ; ArrayforICR(2)];


% rotational speeed of right front wheel with respect to left front wheel in revolutions per minute
DS = 0
