init;

% M is the map with size 5x3 where M(i,:) = [beacon_id, x, y]
M = get_map();

% the total number of steps
num_steps = get_numsteps();

% at t=1 the robot starts at x=0.4060, y=0.1800, theta=0.1244 rad; 
% mu is 3x1
mu = [0.4060;0.1800;0.1244];

% you can choose sigma at t=1
Sigma = diag([0.1 0.1 5*pi/180]).^2; 

% estimate the pose of the robot at each time step and store it in trj_est 
trj_est = zeros(num_steps,3);

%The first pose is known
trj_est(1,:) = mu;


% initliase the R matrix
R = [0 , 0 ; 0 , 0.0140];
% initliase the Q matrix
Q = [0.0420 , 0 ; 0 , 49.0];

% For mean / mu
xt = mu;

% for std
S = Sigma;

% The map
map = M(:,2:3);
       
% Grader ticks function
old_ticks = get_encoders(1);

for t = 2:num_steps
    
    new_ticks = get_encoders(t);

    [d,dth] = get_odom(new_ticks,old_ticks);
    old_ticks = new_ticks;
    
    [xt,S] = predict_step(xt,S,d,dth,R);
    
    I = get_image(t);
    
    Z = sense(I);
    z = Z;
    
    [x,S] = update_step(M,z,xt,S,Q);
    
    trj_est(t,:) = x';
    
end
% write your helper functions below

function [xt,S] = predict_step(xt,S,d,dth,R)
    
    % From the lecture slides the matrices
    xt = move(xt, d, dth);
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

    for i = 1 : size(z,1) % for the length of the map
        
        xland = map(i, 1);
        yland = map(i, 2);

        xroam = x(1);
        yroam = x(2);
        thetaroam = x(3);
        
        % splitting the equations for readability
        sqrt1 = (xland - xroam )^2;
        sqrt2 = (yland - yroam )^2;
        range = (sqrt1 + sqrt2)^0.5;
        
        Bearing = atan2(yland - yroam , xland - xroam) - thetaroam; 
        
        % the G matrix
        row1col1 = -(xland-xroam)/range;
        row1col2 = -(yland-yroam)/range;
        row2col1 = (yland-yroam)/range^2;
        row2col2 = -(xland-xroam)/range^2;
        G = [row1col1 , row1col2, 0;
             row2col1 , row2col2, -1];
        
        % Seperating K calculation for readability
        KPart1 = S*G';
        KPart2 = G*S*G' + Q;
        K_t = KPart1 * (KPart2)^-1;
        err = (z(i,2:3) - [range , wrapToPi(Bearing)]);
        err = [err(1) ; wrapToPi(err(2)) ];
        XValues = x;
        XValues = XValues + K_t * err;
        x = XValues;
        mutiplyS = eye(3) - K_t*G;
        S = mutiplyS * S;
        
    end
    
end    

% From previous prac function
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

% Sense function works with the ones from previous prac
function Z = sense(I)
% input: I is an RGB image of the robot's view
% output: Z = 1xn where n is the number of beacons in the image.
    RGB = I;

    % For Blue mask
    [BWB,maskedRGBImage] = ForBlueColor(RGB);
    s_Blue = regionprops(BWB,'centroid');
    %Areas_Blue = regionprops(BWB,'Area');
    %AreaOfBlueCentroids = Areas_Blue.Area;
    centroids_Blue = cat(1,s_Blue.Centroid);


    % For Red mask
    [BWR,maskedRGBImage] = ForRedColor(RGB);
    s_Red = regionprops(BWR,'centroid');
    %Areas_Red = regionprops(BWR,'Area');
    %AreaOfRedCentroids = Areas_Red.Area;
    centroids_Red = cat(1,s_Red.Centroid);


    % For yellow mask
    [BWY,maskedRGBImage] = ForYellowColor2(RGB);
    s_Yellow = regionprops(BWY,'centroid');
    %Areas_Yellow = regionprops(BWY,'Area');
    %AreaOfYellowCentroids = Areas_Yellow.Area;
    centroids_Yellow = cat(1,s_Yellow.Centroid);

    % Add a tolerance to the blobs to identify the beacons
    MINDIST = 3;


    
    idsAllY = [];
%     centres_YCoord = [centroids_Red(:,2) ,centroids_Yellow(:,2) , centroids_Blue(:,2)];
    for counter = 1 : size(centroids_Blue,1)
        
        [min_X_dist_red,idx_x_red] = min(abs(centroids_Blue(counter,1) - centroids_Red(:,1)));
        [min_X_dist_yellow,idx_x_yel] = min(abs(centroids_Blue(counter,1) - centroids_Yellow(:,1)));
        
        if( min_X_dist_yellow < MINDIST && min_X_dist_red < MINDIST )
            
            [~, idsY] = sort( [   centroids_Red(idx_x_red,2)   centroids_Blue(counter,2)   centroids_Yellow(idx_x_yel,2)  ],'descend');
            idsAllY = [idsAllY; idsY];
        end
    end

    

    
    str = strings;
    
    for counting = 1: size(idsAllY,1)
        for counting2 = 1: size(idsAllY,2)
            
            if idsAllY(counting,counting2) == 3
                str(counting, counting2) = "11"; % yellow
            end
            
            if idsAllY(counting,counting2) == 1 %red
                %str = [str ,[0 1] ];
                str(counting, counting2) = "01";
            end
            
            if idsAllY(counting,counting2) == 2 % blue
                %str = [str , [1 0] ];
                str(counting, counting2) = "10";
            end
            
        end
        
    end
    
    
    % Joining the three centroids
    Joined_Img = (BWB+BWR+BWY);
    %imshow(Joined_Img);
    SE = strel('disk' , 5);
    JoinedImg_Dil = imdilate(Joined_Img, SE);
    imshow(JoinedImg_Dil);

    
    % Using the distance to object equation in teams
    
    Angle_of_view = 62.2; % Horizontal
    focal_length = 3.6/1000; % m % change to 3.04
    camera_sensor_height = 2.64/1000; % m %2.64
    real_height = 150/1000; % m

    %originalImage = imread(fullFileName);
    [rows, columns, numberOfColorChannels] = size(RGB);
    image_height =  rows;
    %rprops = regionprops(RGB,'BoundingBox');
    %object_height = [rprops.BoundingBox];
    
    % Using blob analysis
    Hblob = vision.BlobAnalysis;
    Logical_Image_RGB = logical(JoinedImg_Dil);
    [area, centroid, bbox] = Hblob(Logical_Image_RGB);
    object_height = bbox(:,4);

    % Distance to object
    EqnNancy = focal_length * real_height * image_height;
    EqnDonkey = double(object_height) * camera_sensor_height;
    Dist_To_Object = EqnNancy ./ EqnDonkey;
    
    % Getting the bearing
    % theta = atand( pixel_length * number_of_horizontal_pixels_to_object ) / focal_length ; 
    
    
    V_es = Angle_of_view * 0.5; % V_es is half the camera view angle
    Vx = columns * 0.5; %Half the image width 
    
    blobcenter = centroid(:,1);
    Ix = Vx - blobcenter;
    Bearing_deg = Ix / Vx * V_es;
    

    
    if str == ""
        Z = [];
        return
    end
    
     for i = 1:size(str,1)
         arr(i) = strjoin(str(i,:),'');
     end
     
     for i = 1:size(str,1)
         Z(i) = bin2dec((arr(i)));
     end
    
    %load map1r1.mat;
    %if Z(i) == data(1).beacon(:,1)
    %    plot(Dist_To_Object , data(1).beacon(3:4,2) )
    %end

    % Z(i , :) = [id range (meters) bearing (degrees)]
    Z = [Z', Dist_To_Object, Bearing_deg];

end




function [BWB,maskedRGBImage] = ForBlueColor(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 30-Apr-2021
%------------------------------------------------------

% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.482;
channel1Max = 0.715;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.587;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.000;
channel3Max = 1.000;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BWB = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BWB,[1 1 3])) = 0;

ErodedImg = imopen(BWB , ones(4));
BWB = ErodedImg;

end


function [BWY,maskedRGBImage] = ForYellowColor2(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 30-Apr-2021
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.085;
channel1Max = 0.258;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.384;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.293;
channel3Max = 0.797;

% Create mask based on chosen histogram thresholds
sliderBW = (I(:,:,1) >= channel1Min ) & (I(:,:,1) <= channel1Max) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BWY = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BWY,[1 1 3])) = 0;

ErodedImg = imopen(BWY , ones(4));
BWY = ErodedImg;

end



function [BWR,maskedRGBImage] = ForRedColor(RGB)
%createMask  Threshold RGB image using auto-generated code from colorThresholder app.
%  [BW,MASKEDRGBIMAGE] = createMask(RGB) thresholds image RGB using
%  auto-generated code from the colorThresholder app. The colorspace and
%  range for each channel of the colorspace were set within the app. The
%  segmentation mask is returned in BW, and a composite of the mask and
%  original RGB images is returned in maskedRGBImage.

% Auto-generated by colorThresholder app on 30-Apr-2021
%------------------------------------------------------


% Convert RGB image to chosen color space
I = rgb2hsv(RGB);

% Define thresholds for channel 1 based on histogram settings
channel1Min = 0.973;
channel1Max = 0.030;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.645;
channel2Max = 1.000;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.219;
channel3Max = 0.851;

% Create mask based on chosen histogram thresholds
sliderBW = ( (I(:,:,1) >= channel1Min) | (I(:,:,1) <= channel1Max) ) & ...
    (I(:,:,2) >= channel2Min ) & (I(:,:,2) <= channel2Max) & ...
    (I(:,:,3) >= channel3Min ) & (I(:,:,3) <= channel3Max);
BWR = sliderBW;

% Initialize output masked image based on input image.
maskedRGBImage = RGB;

% Set background pixels where BW is false to zero.
maskedRGBImage(repmat(~BWR,[1 1 3])) = 0;

ErodedImg = imopen(BWR , ones(4));
BWR = ErodedImg;


end



