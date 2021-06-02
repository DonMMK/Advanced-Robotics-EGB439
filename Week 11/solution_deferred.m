close all;clear;clc
load map1r3.mat

M = [
    30, 0.316, 1.856;
    27, 1.716, 1.74;
    57, 0.272, 0.708;
    45, 0.944, 0.18;
    39, 1.788, 0.216;
    ];
num_steps = length(data);
%init;

% M is the map with size 5x3 where M(i,:) = [beacon_id, x, y]
%M = get_map();

% the total number of steps
%num_steps = get_numsteps();

% at t=1 the robot starts at x=0.4060, y=0.1800, theta=0.1244 rad; 
% mu is 3x1
%mu = [0.4060;0.1800;0.1244];
mu = [data(1).pose.x; data(1).pose.y ; data(1).pose.theta];

% you can choose sigma at t=1
Sigma = diag([0.1 0.1 5*pi/180]).^2; 

% estimate the pose of the robot at each time step and store it in trj_est 
trj_est = zeros(num_steps,3);

%The first pose is known
trj_est(1,:) = mu;

% initliase the R matrix
R =[0 , 0 ; 0 , 0.014] ;
% initliase the Q matrix
Q =[0.041 , 0 ; 0 , 49] ;

% For mean / mu
xt = mu;

% for std
S = Sigma;

% The map
map = M(:,2:3);
       
% Grader ticks function
old_ticks = [0,0];


%% Main
for t = 2:num_steps
    
    new_ticks = data(t).encoders;

    [d,dth] = get_odom(new_ticks,old_ticks);
    old_ticks = new_ticks;
    
    % calling the predict step
    [xt,S] = predict_step(xt,S,d,dth,R);
    
    %I = get_image(t);
    I = data(t).image;
    Z = sense(I);
    z = Z;
    
    % check
    z = zeros(5, 2);
    if ~isempty(Z)
        [~,idx] = ismember(Z(:,1),M(:,1),'rows');
        ConditionPasss = Z(:,2:3);
        z(idx,:) = ConditionPasss;
    end
    
    % calling the update step
    
    [xt,S] = update_step(M(:,2:3),z,xt,S,Q);
    
    transpose_t = xt';
    trj_est(t,:) = transpose_t
    
end




%% Helper functions mmmm
function [xt,S] = predict_step(xt,S,d,dth,R)
    
    theta = xt(3);
    
    one = 1;
    zero = 0;
    row1col3 = -d*sin(theta);
    row2col3 = d*cos(theta);
    Jacobian_x = [one, zero , row1col3;
                  zero , one , row2col3;
                  zero , zero , one];
    
    rrow1col1 = cos(theta);
    rrow2col1 = sin(theta);
    
    Jacobian_u = [rrow1col1 zero;
                  rrow2col1 zero; 
                  zero , one];
    
    xt = xt + [ d*cos(theta) ;
                    d*sin(theta); 
                    dth];
    
    % Seperating the steps for readability
    Step1_S = Jacobian_x * S * Jacobian_x'; 
    Step2_S = Jacobian_u * R * Jacobian_u';
    S = Step1_S + Step2_S;
    
end

function [x,S] = update_step(map,z,x,S,Q)
    forcondition = 1 : length(map);
    for i = forcondition  % for the length of the map
        if z(i,1) == 0
            continue
        end
        theta_value = x(3);
        yr_value = x(2);
        xr_value = x(1);
        
        yl_map = map(i,2);
        xl_map = map(i,1);
        
        range_part1 = (xl_map-xr_value)^2;
        range_part2 = (yl_map-yr_value)^2;
        range = sqrt(range_part1 + range_part2);
        
        %bearing1 = atan2(yl_map-yr_value,xl_map-xr_value);
        %bearing = wrapToPi(bearing1 - theta_value);
        
        
        % Change from here
        GMatR1C1 = -(xl_map-xr_value)/range;
        GMatR1C2 = -(yl_map-yr_value)/range ;
        GMatR2C1 = (yl_map-yr_value)/(range^2);
        GMatR2C2 = -(xl_map-xr_value)/(range^2);
        
        G = [GMatR1C1 GMatR1C2 0;
            GMatR2C1 GMatR2C2 -1];
        KPart1 = (S*G');
        KPart2 = (G*S*G' + Q);
        K = KPart1/KPart2;
        
        Change_diff = (z(i,:)'-range);
        
        matt1 = Change_diff(1);
        matt2 =  wrapToPi(Change_diff(2));
        Change_diff = [matt1;matt2];
            
        % change to match 
        x_add = K*Change_diff;
        x = x + x_add;
        I = eye(3);
        
        SMultiply = (I - K*G);
        S = SMultiply*S; 
        
    end
    
end 

% not used
function next_pose = move(current_pose, d , dth)
%inputs: current_pose is a 3x1 vector [x; y; theta] of the robot (theta in radians)
%        d  distance traveled in meters
%        dt angle rotated in radians 
%outputs: next_pose is a 3x1 vector [x; y; theta] of the robot (theta in radians)
    x_dash = current_pose(1,:) + d * cos(current_pose(3,:));
    y_dash = current_pose(2,:) + d * sin(current_pose(3,:));
    theta_dash = current_pose(3,:) + dth;
    
    % Calculate the next pose for move function
    next_pose = [x_dash; y_dash; theta_dash];



end

function [d , dth] = get_odom(new_ticks,old_ticks)
%inputs: new_ticks , old_ticks both are 1x2 vectors [left_ticks right_ticks]
%outputs: d  distance traveled in meters
%        dt angle rotated in radians 

    % Using the values from the georgia tech video
    Num_tick_per_rot = 370;
    Wheel_diameter = 0.065;
    Wheel_axis = 0.15;
    
    delta_tick_left = new_ticks(:,1) - old_ticks(:,1);
    delta_tick_right = new_ticks(:,2) - old_ticks(:,2);
    
    D_left = (2 * pi * 0.5 * Wheel_diameter * delta_tick_left) / Num_tick_per_rot;
    D_right= (2 * pi * 0.5 * Wheel_diameter * delta_tick_right) / Num_tick_per_rot;
    
    % Calculation for the values
    d = ( D_left + D_right ) / 2;
    dth =  (D_right- D_left)/ Wheel_axis;
    



end

%% Helper functions nnnnmmmm
function Z = sense(I)
    RGB = I;
    
    %BW >> objects >> centroids >> bb >> areas >> b >>
    
    % Sense function from previous pracs updated
    [BWB , ~] = ForBlueColor2(RGB);
    s_Blue = regionprops(BWB,'Centroid','Area','BoundingBox');
    
    
    % Sense function from previous pracs updated
    [BWR , ~] = ForRedColor2(RGB);
    s_Red = regionprops(BWR,'Centroid','Area','BoundingBox');
    
    
    % Sense function from previous pracs updated
    [BWY , ~] = ForYellowColor2(RGB);
    s_Yellow = regionprops(BWY,'Centroid','Area','BoundingBox');
    
    % blue
    centroids_Blue = cat(1,s_Blue.Centroid);
    Blue_BBox = cat(1,s_Blue.BoundingBox);
    Area_Blue = cat(1,s_Blue.Area);
    
    % red
    centroids_Red = cat(1,s_Red.Centroid);
    Red_BBox = cat(1,s_Red.BoundingBox);
    Area_Red = cat(1,s_Red.Area);
    
    % yellow
    centroids_Yellow = cat(1,s_Yellow.Centroid);
    Yellow_BBox = cat(1,s_Yellow.BoundingBox);
    Area_Yellow = cat(1,s_Yellow.Area);
    
    % Binary for colors
    YYString = '11' ; 
    YY = YYString;
    RRString = '01' ;
    RR = RRString;
    BBString = '10' ;
    BB = BBString;
    
    Z = [];
    
    conditionIF = isempty(Area_Blue)==1 || isempty(Area_Red) ==1 || isempty(Area_Yellow) ==1;
    if conditionIF 
        Z = [];
    else
        for index = 1:2
            Beacon_Blue_PassValue = [10,centroids_Blue(index,:)];
            Beacon_Blue = Beacon_Blue_PassValue;
            
            Beacon_Red_PassValue = [01,centroids_Red(index,:)];
            Beacon_Red = Beacon_Red_PassValue;
            
            Beacon_Yellow_PassValue = [11,centroids_Yellow(index,:)];
            Beacon_Yellow = Beacon_Yellow_PassValue;
            
            Bea_con = [Beacon_Blue ; Beacon_Red; Beacon_Yellow];
            Beacons = Bea_con;
            Beacons = sortrows(Beacons , 3, 'descend');
            
            BinaryString = '';
            BSS = BinaryString;
            oneone = 11;
            onezero = 10;
            zeroone = 01;
            for index2 = 1:3
                if Beacons(index2 , 1) == oneone
                    BSS = append(BSS , YY);
                    
                elseif Beacons(index2 , 1) == onezero
                    BSS = append(BSS , BB);
                    
                elseif Beacons(index2 , 1) == zeroone
                    BSS = append(BSS , RR);
                    
                end
                
            end
            
            % Bin to dec 
            PassBeaconsBin = bin2dec(BSS);
            BeaconBinary = PassBeaconsBin;
            
            
            Beacon_Height = [Yellow_BBox(index,:) ; Blue_BBox(index,:) ; Red_BBox(index,:) ];
            UnSortedBH = Beacon_Height;
            
            Sorted_Height = sortrows(UnSortedBH, 2, 'descend');
            % CC
            TopPartSortEqu = Sorted_Height(1, 2) - Sorted_Height(3, 2);
            Sorted_Height_In = TopPartSortEqu + Sorted_Height(1, 4);
            
            
            % For range and bearing prac
            cent1 = centroids_Blue(index,:);
            cent2 = centroids_Yellow(index,:);
            cent3 = centroids_Red(index,:) ;
            Centroids_Unsorted = [ cent1; cent2 ; cent3 ];
            Centroid_All = Centroids_Unsorted;
            Sorted_Centroids = sortrows(Centroid_All,1);
            
            Sorted_Center = (Sorted_Centroids(3,1) + Sorted_Centroids(2,1) + Sorted_Centroids(1,1)) ./ 3; 
            
            % Using the distance to object equation in teams
    
            Angle_of_view = 62.2; % Horizontal
            focal_length = 3.6; % m % change to 3.04
            camera_sensor_height = 2.75; % mm %2.64
            real_height = 150; % mm
            
            Range = ((focal_length * real_height * 240)/ (Sorted_Height_In*camera_sensor_height) ) * 0.001;
            Bearing = (320/2-Sorted_Center)/(320/2)*(Angle_of_view/2);
            
            PassingZ = [BeaconBinary ,Range , Bearing ];
            NewZ = PassingZ;
            Z = [NewZ ; Z];
            
            if length(Area_Yellow) ==1|| length(Area_Blue)==1 || length(Area_Red) ==1
                break;
            end
        end
    end
    
    
    
    
end


%% Masks for the sense function
function [BWY,maskedRGBImage] = ForYellowColor2(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 24-May-2021
%------------------------------------------------------
% Convert RGB image to chosen color space
I = RGB;
% Define thresholds for channel 1 based on histogram settings
channel1Min = 100.000;
channel1Max = 199.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 101.000;
channel2Max = 187.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 1.000;
channel3Max = 88.000;


% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BWY = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BWY,[1 1 3])) = 0;
%attempt to remove small. same as using eroded in previous attempts
BWY = medfilt2(BWY, [5 5]);
end


function [BWB,maskedRGBImage] = ForBlueColor2(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 24-May-2021
%------------------------------------------------------

% Convert RGB image to chosen color space
I = RGB;

% Define thresholds for channel 1 based on histogram settings
channel1Min = 13.000;
channel1Max = 43.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 15.000;
channel2Max = 80.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 57.000;
channel3Max = 126.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BWB = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BWB,[1 1 3])) = 0;
%attempt to remove small. same as using eroded in previous attempts
BWB = medfilt2(BWB, [5 5]);
end


function [BWR,maskedRGBImage] = ForRedColor2(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 24-May-2021
%------------------------------------------------------


% Convert RGB image to chosen color space
I = RGB;

% Define thresholds for channel 1 based on histogram settings
channel1Min = 71.000;
channel1Max = 180.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 1.000;
channel2Max = 53.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 1.000;
channel3Max = 72.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BWR = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BWR,[1 1 3])) = 0;
%attempt to remove small. same as using eroded in previous attempts
BWR = medfilt2(BWR, [5 5]);
end

