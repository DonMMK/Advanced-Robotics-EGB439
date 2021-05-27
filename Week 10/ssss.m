% In Grader, we will load the data for you
% init;
close all;clear;clc
% when developing offline, you need to load one of the data files 
load map1r3.mat

% In Grader, we give you the map
% M is the map with size 5x3 where M(i,:) = [beacon_id, x, y]
% M = get_map();

% When developing and testing offline, you need to choose the correct map
% that matches your data.
% Map 1
M = [
    30, 0.316, 1.856;
    27, 1.716, 1.74;
    57, 0.272, 0.708;
    45, 0.944, 0.18;
    39, 1.788, 0.216;
    ];

% Map 2
% M = [
%     39, 0.968, 1.792;
%     30, 0.252, 1.336;
%     57, 0.264, 0.364;
%     27, 1.604, 0.208;
%     45, 1.832, 1.384;
%     ];

% In Grader, we give you the number of steps. 
% num_steps = get_numsteps();
% When developing offline, the number of steps is the length of the run
% you loaded 
num_steps = length(data);

% In Grader, at t=1 the robot starts at x=0.4060, y=0.1800, theta=0.1244 rad; 
% mu is 3x1
% mu = [0.4060;0.1800;0.1244];
% When developing offline, mu at time t = 1 is the pose from the first time step
mu = [data(1).pose.x; data(1).pose.y ; data(1).pose.theta];

% you can choose sigma at t=1
Sigma = diag([0.1 0.1 5*pi/180]).^2; 

% estimate the pose of the robot at each time step and store it in trj_est 
trj_est = zeros(num_steps,3);

%The first pose is known
trj_est(1,:) = mu;


%%

% Get R and Q value for predict step
R = diag([.01 10*pi/180]).^2;
Q = diag([.009 10*pi/180]).^2;
R=R*0.5*10;
Q(1) = 0.042;
R(1) = 0;
Q(4) = 49;
R(4) = 0.0140;

% Obtain car specification from Prac 9
N = 370;
wheelRadius = 0.065/2;
wheelAxis = 0.15;

% Get last encoder matrix
oldticks = [0 0];

for t =2:num_steps
    % move
    % In Grader, you have to use the function get_encoders(time_step).
    new_ticks = data(t).encoders;    
    % when working offline, use 
%     new_ticks = data(t).encoders;
    
    % Get odom from prac 9
    dt = new_ticks - oldticks;
    oldticks = new_ticks;
    
    dist = 2 * pi * wheelRadius * (dt/N);
    
    d = sum(dist)/2;
    dth = (dist(2) - dist(1))/wheelAxis;
    
    [mu, Sigma] = predict_step(mu, Sigma, d, dth, R);

    % sense
    % In Grader, you get the image using the function get_image(time_step);
    I = data(t).image;
    % when working offline you use
%     I = data(t).image;
     Z = sense(I);
     
    z = zeros(5, 2);
    if ~isempty(Z)
        [~,idx] = ismember(Z(:,1),M(:,1),'rows');
        z(idx,:) = Z(:,2:3);
    end 
    
    % Update step 
    [mu, Sigma] = update_step(M(:,2:3), z, mu, Sigma, Q);
    plot_cov(mu,Sigma,3)
    
    % Update new trj
    trj_est(t,:) = mu
end 

% write your helper functions below

function [xt,S] = predict_step(xt,S,d,dth,R)
    % page 34/45 Module 4
    % Lecture Topic 3.3-Extended KF
    
    % Jacobian matrix 
    Jx = [1 0 -d*sin(xt(3));
        0 1 d*cos(xt(3));
        0 0 1];
    Ju = [cos(xt(3)) 0;
        sin(xt(3)) 0;
        0 1];
    
    xt = xt + [d*cos(xt(3)); d*sin(xt(3));dth];
    
    S = Jx*S*Jx' + Ju*R*Ju';
end

   
% This function takes:
%     The location of all the landmarks (map)
%     The sensor readings of the range and bearing to all the landmarks in the map at the current time step.
%     The mean, and a covariance matrix of the robot pose
%     and the matrix Q (the covariance of the sensor noise). 
%
% The function performs an update step of the EKF localiser and returns the mean and covariance of the robot. 
function [x,S] = update_step(map,z,x,S,Q)
    for i = 1:length(map)
        if z(i,1) == 0
            continue
        end
        
        
        % Lecture Topic 3.3-Extended KF
        % calculate range and bearing
        % get land mark and co-ordinations
        xl = map(i,1);
        yl = map(i,2);
        
        xr = x(1);
        yr = x(2);
        theta_r = x(3);
        
        r = sqrt((xl-xr)^2 + (yl-yr)^2);
        % wrapToPi for subtracting angles
        B = wrapToPi(atan2(yl-yr,xl-xr) - theta_r);
        
        G = [-(xl-xr)/r -(yl-yr)/r 0;
            (yl-yr)/(r^2) -(xl-xr)/(r^2) -1];
        
        K = (S*G')/(G*S*G' + Q);
        
        diff = (z(i,:)'-r);
        % wrapToPi for subtracting angles
        diff = [diff(1); wrapToPi(diff(2))];
            
        % change to match 
        x = x + K*diff;
        I = eye(3);
        S = (I - K*G)*S;
    end
end   

function Z = sense(I)
% for week 7 task, detect IDs only
% input: I is an RGB image of the robot's view
% output: Z = 1xn where n is the number of beacons in the image.

%for week 8 task, detect IDs and estimate range and bearing
% input: I is an RGB image of the robot's view
% output: Z = nx3 where Z(i,:) = [id range bearing] (bearing should be in degrees)

%only BW imgs are needed
[red_BW,~] = red_createMask(I);
[blue_BW,~] = blue_createMask(I);
[yellow_BW,~] = yellow_createMask(I);

%find objects
red_s = regionprops(red_BW, 'Centroid', 'Area', 'BoundingBox');
blue_s = regionprops(blue_BW, 'Centroid', 'Area', 'BoundingBox');
yellow_s = regionprops(yellow_BW, 'Centroid', 'Area', 'BoundingBox');

%find centroids
red_centroids = cat(1,red_s.Centroid);
blue_centroids = cat(1,blue_s.Centroid);
yellow_centroids = cat(1,yellow_s.Centroid);

%find bounding box
red_bbox = cat(1,red_s.BoundingBox);
blue_bbox = cat(1,blue_s.BoundingBox);
yellow_bbox = cat(1,yellow_s.BoundingBox);

% find areas
red_area = cat(1, red_s.Area);
blue_area = cat(1, blue_s.Area);
yellow_area = cat(1, yellow_s.Area);

%binary code for each colour
r = '01';
b = '10';
y = '11';

Z=[];
    % Calculate the binary code
    if isempty(red_area) == 1 || isempty(yellow_area) == 1 || isempty(blue_area) == 1
        Z = []; % Nothing from the image
    
    else
        for i = 1:2
            redBeacon = [01, red_centroids(i,:)];
            yellowBeacon = [11, yellow_centroids(i,:)];
            blueBeacon = [10, blue_centroids(i,:)];
            
            beacons = [blueBeacon; redBeacon; yellowBeacon];
            beacons = sortrows(beacons, 3, 'descend');
            
            bins = '';
            for ii = 1:3
                if beacons(ii, 1) == 10
                    bins = append(bins, b);
                elseif beacons(ii, 1) == 01
                    bins = append(bins, r);
                elseif beacons(ii, 1) == 11
                    bins = append(bins, y);
                    
                end
                
            end
            
            % But the beacon into decimal 
            beaconBins = bin2dec(bins);
            %Z = [Z; beaconCode]';
            
            % Estimate the range by obtaining the height of the object's
            % image
            heights = [red_bbox(i,:); blue_bbox(i,:); yellow_bbox(i,:)];
            sortedHeights = sortrows(heights, 2, 'descend'); % need to sort height
            img_object_h = sortedHeights(1, 2) - sortedHeights(3, 2) + sortedHeights(1, 4);
            
            % Estimate the bearing by getting the centroid of the object's
            % image
            centroids = [red_centroids(i,:); yellow_centroids(i,:); blue_centroids(i,:)];
            sortedCentroids = sortrows(centroids, 1);
            % obtain the middle object's centroid, which is the centroid of
            % the object's image
            img_object_x = (sortedCentroids(3,1) + sortedCentroids(2,1) + sortedCentroids(1,1)) ./ 3; 
            
            % Get the range and bearing values
            % remove the function here
            [range, bearing] = GetRangeAndBearing(img_object_h, img_object_x);
            
            % Obtain Z
            newZ = [beaconBins range bearing];
            Z = [newZ; Z];
            
            if length(red_area) == 1 || length(yellow_area) == 1 || length(blue_area) == 1
                break;
            end
        end
    end
end
% write all your helper functions below and call them inside the function sense

% calculate range and bearing
function [range, bearing] = GetRangeAndBearing(img_object_h, img_object_x)
    % Camera specs
    img_w = 320; % pixels
    img_h = 240; % pixels
    aov = 62.2; % degree, angle of view
    f = 3.6; % mm
    sensor_h = 2.74; %mm
    real_object_h = 150; %mm
    
    % apply the formula
    % https://photo.stackexchange.com/questions/102795/calculate-the-distance-of-an-object-in-a-picture
    % https://www.scantips.com/lights/fieldofviewmath.html
    range = (f*real_object_h*img_h)/(img_object_h*sensor_h); %mm
    range = range*10^(-3); % convert to m
    
    % find bearings
    % https://teams.microsoft.com/l/message/19:c638fa7ba24546e5b2343c51cc6d0f81@thread.tacv2/1620086758563?tenantId=dc0b52a3-68c5-44f7-881d-9383d8850b96&groupId=eb197ae4-597b-4870-86f4-ee37a20aab06&parentMessageId=1620086758563&teamName=EGB439_2021%3AAdvanced%20Robotics&channelName=Module%203%20-%20Bayesian%20filtering%20and%20localisation&createdTime=1620086758563
    % trigonemetry
    bearing = (img_w/2-img_object_x)/(img_w/2)*(aov/2);
end

% auto-generated by color thresholder
function [BW,maskedRGBImage] = red_createMask(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 28-Apr-2021
%------------------------------------------------------


% Convert RGB image to chosen color space
I = RGB;

% Define thresholds for channel 1 based on histogram settings
channel1Min = 72.000;
channel1Max = 179.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.000;
channel2Max = 52.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.000;
channel3Max = 73.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

% filter pepper noise
BW = medfilt2(BW, [5 5]);
end

function [BW,maskedRGBImage] = blue_createMask(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 28-Apr-2021
%------------------------------------------------------


% Convert RGB image to chosen color space
I = RGB;

% Define thresholds for channel 1 based on histogram settings
channel1Min = 14.000;
channel1Max = 42.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 16.000;
channel2Max = 79.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 58.000;
channel3Max = 125.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

% filter pepper noise
BW = medfilt2(BW, [5 5]);
end

function [BW,maskedRGBImage] = yellow_createMask(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 28-Apr-2021
%------------------------------------------------------


% Convert RGB image to chosen color space
I = RGB;

% Define thresholds for channel 1 based on histogram settings
channel1Min = 99.000;
channel1Max = 198.000;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 102.000;
channel2Max = 186.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.000;
channel3Max = 87.000;


% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BW = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BW,[1 1 3])) = 0;

% filter pepper noise
BW = medfilt2(BW, [5 5]);
end
