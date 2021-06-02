close all;clear;clc
% when developing offline, you need to load one of the data files 
load map1r1.mat


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

% estimate the position of the beacons and stor it in M_est
% M_est(i,:) = [beacon_id, beacon_x,beacon_y,variance_x,variance_y,covariance_xy]. 
M_est = zeros(5,6);

%The first pose is known
trj_est(1,:) = mu;

% Initialize Q and R
Q = diag([0.5 20*pi/180]).^2;
R = diag([0.04 40*pi/180]).^2;

% initialize M_est
M_est = zeros(5,6);
M_est(:,1) = [30; 27; 57; 45; 39];
beacon_ids = M_est(:,1);

% set a starting array
starting_array = zeros(1, 5);

% init 
old_ticks  = [0 ,0];

for t =2:num_steps
    % move
    new_ticks = data(t).encoders;
    

    [d,dth] = get_odom(new_ticks,old_ticks);
    old_ticks = new_ticks;
    
    
    [mu,Sigma] = predict_step(mu,Sigma,d,dth,R);
    % sense
    %I = get_image(t);
    I = data(t).image;
    
    % Check code
    Z = sense(I);
    
    z = zeros(5, 2);
    
    if ~isempty(Z)
        [~,index] = ismember(Z(:,1),beacon_ids,'rows');
        z(index,:) = Z(:,2:3);
        
        for j = 1:length(index)
            if starting_array(index(j))
                [mu,Sigma] = update_step(index(j), z(index(j),:), Q, mu, Sigma);
            
            elseif ~starting_array(index(j))
                [mu, Sigma] = initLandmarks(z, Q, mu, Sigma);
                
                starting_array(index(j)) = 1;
            end
        end
    end 

    trj_est(t,:) = mu(1:3);
    for j = 1:size(M_est,1)
        M_est(j,2:3) = mu(2*j+2:2*j+3); 
        out = Sigma(2*j+2:2*j+3,2*j+2:2*j+3);
        M_est(j,4) = out(1);
        M_est(j,5) = out(end);
        M_est(j,6) = out(2)
    end 
    
end
% write your helper functions below


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

function [mu, Sigma] = update_step(landmarkID,zi,Q,mu,Sigma)
   
    
    
    X_Value = mu(1);
    Y_Value = mu(2);
    i = landmarkID * 2 - 1 + 3;
    indexing = i;
    rangePart1 = (X_Value - mu(indexing))^2 ;
    rangePart2 = (Y_Value - mu(indexing + 1))^2; 
    range = sqrt(rangePart1 + rangePart2); 
    bearing = atan2( mu(i+1) - Y_Value , mu(i) - X_Value ) - mu(3);
    
    error = zi - [range , bearing];
    err = error;
    err = [err(1); wrapToPi(err(2))];
    
    M3Row1Col1 = -(mu(i)-X_Value)/range;
    M3Row1Col2 = -(mu(i+1)-Y_Value)/range;
    M3Row2Col1 = (mu(i+1)-Y_Value)/range^2;
    M3Row3Col2 = -(mu(i)-X_Value)/range^2;
    Garray = [M3Row1Col1 , M3Row1Col2 , 0;
              M3Row2Col1 , M3Row3Col2 , -1];
          
    ZeroG = zeros(2 ,20);
    G = [Garray , ZeroG];
    
    
    G3R1C1 = (mu(i) - X_Value)/range;
    G3R1C2 = (mu(i+1)- Y_Value)/range;
    G3R2C1 = -(mu(i+1)- Y_Value)/range^2;
    G3R2C2 = (mu(i)- X_Value)/range^2 ;
    GLast = [G3R1C1 ,G3R1C2;
              G3R2C1, G3R2C2 ];
          
    G(:,i:i+1) = GLast;
          
    K1 = Sigma * G';
    K11 = K1;
    K2 = G*Sigma*G' + Q;
    K22 = K2;
    K = K11/K22;

    mu = mu + K*(err);
    Sigma = (eye(23)-K*G)*Sigma;
    
end

function [mu, Sigma] = initLandmarks(z,Q,mu,Sigma)

    ArrayStore = [];
    ArrayStoreMU = [];
    
    for i = 1 : length(z)
        M4Part1 = mu(1) + z(i,1)*cos(mu(3)+z(i,2));
        M4Part2 = mu(2) + z(i,1)*sin(mu(3)+z(i,2)); 
        
        NewArray = [ M4Part1; M4Part2 ];
        
        ArrayStoreMU = [ArrayStoreMU; NewArray];
        
        M5R1C1 = cos(mu(3)+z(i,2));
        M5R1C2 = -z(i,1)*sin(mu(3)+z(i,2));
        M5R2C1 = sin(mu(3)+z(i,2));
        M5R2C2 = z(i,1)*cos(mu(3)+z(i,2));
        
        MatrixLz = [M5R1C1 , M5R1C2 ; M5R2C1 , M5R2C2];
        
        ArrayStore = blkdiag(ArrayStore,MatrixLz*Q*MatrixLz');
        
    end
    
    
    if size(mu)  == [3 1]
        TempMU = ArrayStoreMU;
        mu = [mu;TempMU];
    else
        TempMU = ArrayStoreMU;
        mu(4:end) = TempMU;
    end

    if size(Sigma) == [3 3]
        PassSig = blkdiag(Sigma,ArrayStore);
        Sigma = PassSig;
    else
        Sigma(4:end,4:end) = ArrayStore;
        
    end   
    
    
end

function [mu,Sigma] =predict_step(mu,Sigma,d,dth,R)
    Matrix1Row1Col1 = d*cos(mu(3)) ;
    Matrix1Row2Col1 = d*sin(mu(3)) ;
    Matrix1Row3Col1 = dth;
    
    mu(1:3) = mu(1:3) + [Matrix1Row1Col1;Matrix1Row2Col1;Matrix1Row3Col1];
    
    Var_One = 1;
    Var_Zero = 0;
    
    Mat2R1C3 = -d*sin(mu(3));
    Mat2R2C3 = d*cos(mu(3));
    Jacobian_X = [1, 0, Mat2R1C3;
                  0, 1, Mat2R2C3;   
                  0, 1, 1];
              
    Mat3R1C1 = cos(mu(3));
    Mat3R2C1 = sin(mu(3));
    Jacobian_U = [ Mat3R1C1, 0;
                   Mat3R2C1, 0;
                   0       , 1];
               
    if size(Sigma) == [3 3]
        Sigma_P1 = Jacobian_X * Sigma * Jacobian_X';
        Sigma_P2 = Jacobian_U *R * Jacobian_U';
        Sigma = Sigma_P1 + Sigma_P2;
    else
        Jacobian_X = blkdiag(Jacobian_X,eye(20));
        Jacobian_U = [Jacobian_U;zeros(20,2)];
        Sigma2_P1 = Jacobian_X*Sigma*Jacobian_X';
        Sigma2_P2 = Jacobian_U*R*Jacobian_U'; 
        Sigma = Sigma2_P1 + Sigma2_P2;
    end
    
    
    
    
end

% Sense + Range and Bearing + Chromaticity
function Z = sense(I)
    [r, g, b] = find_chromacity(I);
    redBW = r > 0.6;
    blueBW = b > 0.4;
    yellowBW = r > 0.4 & g > 0.4;
    
    try
        redblobs = regionprops(redBW);
        blueblobs = regionprops(blueBW);
        yellowblobs = regionprops(yellowBW);
        
        red_areas = cat(1,redblobs.Area);
        blue_areas = cat(1,blueblobs.Area);
        yellow_areas = cat(1,yellowblobs.Area);
        
        red_bboxs = cat(1,redblobs.BoundingBox);
        blue_bboxs = cat(1,blueblobs.BoundingBox);
        yellow_bboxs = cat(1,yellowblobs.BoundingBox);
        
        red_centroids = cat(1,redblobs.Centroid);
        blue_centroids = cat(1,blueblobs.Centroid);
        yellow_centroids = cat(1,yellowblobs.Centroid);
        
        [red_bounds,~,~,~] =  bwboundaries(redBW);
        [blue_bounds,~,~,~] = bwboundaries(blueBW);
        [yellow_bounds,~,~,~] = bwboundaries(yellowBW);
    catch 
        Z = [];
        return
    end
    
    % Check if one blob per colour
    if length(red_areas) < 1 || length(blue_areas) < 1 || length(yellow_areas) < 1
        Z = [];
        return
    end

    Z = [];
    
    red_noise = find(red_areas < 30);
    blue_noise = find(blue_areas < 30);
    yellow_noise = find(yellow_areas < 30);
      
    red_areas(red_noise) = [];
    blue_areas(blue_noise) = [];
    yellow_areas(yellow_noise) = [];
    
    red_bboxs(red_noise,:) = [];
    blue_bboxs(blue_noise,:) = [];
    yellow_bboxs(yellow_noise,:) = [];
    
    red_centroids(red_noise,:) = [];
    blue_centroids(blue_noise,:) = [];
    yellow_centroids(yellow_noise,:) = [];
    
    red_bounds(red_noise) = [];
    blue_bounds(blue_noise) = [];
    yellow_bounds(yellow_noise) = [];
    
    beacon_ids = [27; 30; 39; 45; 57];
    
    for i = 1:2
        [~,red_noise] = max(red_areas);
        [~,blue_noise] = max(blue_areas);
        [~,yellow_noise] = max(yellow_areas);
        
        blobs = [red_bboxs(red_noise,2) blue_bboxs(blue_noise,2) yellow_bboxs(yellow_noise,2)];
        heights = [red_bounds(red_noise); blue_bounds(blue_noise); yellow_bounds(yellow_noise)];
        centroids = [red_centroids(red_noise,:) ; blue_centroids(blue_noise,:) ; yellow_centroids(yellow_noise,:)];
        
        [~, ind] = sort(blobs);
        ind = fliplr(ind);
        
        if (isempty(ind) ~= 1)        
            colourString = {'01' '10' '11'};
            bins = cell2mat(colourString(ind));

            dec = bin2dec(bins);
            index = find(beacon_ids==dec);
            
            if (isempty(index) ~= 1)
                top = cell2mat(heights(ind(1)));
                bottom = cell2mat(heights(ind(3)));
                img_height = max(top(:,1)) - min(bottom(:,1))+2;
                img_x = centroids(ind(2),1);
                RangeAndBearing = GetRangeAndBearing(img_height, img_x);
                
                result = zeros(1,3);
                result(1) = dec;
                result(2) = RangeAndBearing(1);
                result(3) = RangeAndBearing(2);
                
                Z = [Z ; result];
            end 
        end 
        
        
        % Check if one blob per colour
        if length(red_areas) < 1 || length(blue_areas) < 1 || length(yellow_areas) < 1
            return
        end
              
    end 
   

end
% write all your helper functions below and call them inside the function sense
function [redChannel,greenChannel,blueChannel] = find_chromacity(img)
    img = double(img);
    imgTotal = img(:,:,1) + img(:,:,2) + img(:,:,3);
    
    % Get red, green and blue
    redChannel = img(:,:,1)./imgTotal;
    greenChannel = img(:,:,2)./imgTotal;
    blueChannel = img(:,:,3)./imgTotal;
    
end

function z = GetRangeAndBearing(height, position)
    w_cam = 320;         
    h_cam = 240;
    f = 3.6 * 10^-3;  
    aov = deg2rad(62.2);   
    h_beacon = 0.15;  
    h_sensor = 2.74 * 10^-3;

    % Calculate distance
    range = h_beacon * f * h_cam / (height * h_sensor) ;

    % Calculate angle
    angle_temp = (position - (w_cam/2)) / (w_cam/2);
    bearing = -(aov/2) * angle_temp;

    % Return values
    z = [range; bearing];

end
