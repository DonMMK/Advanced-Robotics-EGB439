init;

% the total number of steps
num_steps = get_numsteps();

% at t=1 the robot starts at x=0.4060, y=0.1800, theta=0.1244 rad; 
% mu is 3x1
mu = [0.4060;0.1800;0.1244];

% you can choose sigma at t=1
Sigma = diag([0.1 0.1 5*pi/180]).^2; 

% estimate the pose of the robot at each time step and store it in trj_est 
trj_est = zeros(num_steps,3);

% estimate the position of the beacons and stor it in M_est
% M_est(i,:) = [beacon_id, beacon_x,beacon_y,variance_x,variance_y,covariance_xy]. 
M_est = zeros(5,6);

%The first pose is known
trj_est(1,:) = mu;


% Initilaize Q and R

Q = [ 0.32^2, 0;
      0, 0.33^2];

R = [ 0.050^2, 0;
      0, 0.7^2  ];


% Codde

M_est = zeros(5,6);
Beee1 = 30;
Beee2 = 27;
Beee3 = 57;
Beee4 = 45;
Beee5 = 39;
M_est(:,1) = [Beee1; Beee2; Beee3; Beee4; Beee5];
NewM = M_est(:,1);
beacon_ids = NewM;


ArrayInnit = zeros(1, 5);

old_ticks = [0, 0];

for t =2:num_steps

    new_ticks = get_encoders(t);
    % get odom
    [d,dth] = get_odom(new_ticks,old_ticks);
    old_ticks = new_ticks;
    
    
    [mu,Sigma] = predict_step(mu, Sigma, d, dth, R);
    

    I = get_image(t);
    
    Z = sense(I);
    
    zzpass = zeros(5, 2);
    z = zzpass;
    
    if ~isempty(Z)
        ismem1 = Z(:,1);
        ismem2 = beacon_ids;
        ismem3 = 'rows';
        [~,index] = ismember(ismem1,ismem2,ismem3);
        z(index,:) = Z(:,2:3);
        
        JCondit = 1:length(index);
        for j = JCondit
            condittif = ArrayInnit(index(j));
            if condittif
                
                [mu,Sigma] = update_step(index(j), z(index(j),:), Q, mu, Sigma);
                
                % calling the function
            elseif ~ArrayInnit(index(j))
                [mu, Sigma] = initLandmarks(z, Q, mu, Sigma);
                
                ArrayInnit(index(j)) = 1;
            end
        end
    end 
    nn1 = 1;
    nn3 = 3;
    trj_est(t,:) = mu(nn1:nn3);
    
    for j = 1:size(M_est,1)
        nnn2 = 2;
        M_est(j,2:3) = mu( nnn2 * j + nnn2 : nnn2 * j + nn3); 
        
        SigmaLeft = nnn2 * j + nnn2 : nnn2 * j + nn3;
        SigmaRight = nnn2 * j + nnn2 : nnn2 * j +nn3;
        outpass = Sigma( SigmaLeft , SigmaRight);
        out = outpass;
       
       
        mm6   = out(2);
        M_est(j,6) = mm6;
         mm4 = out(1);
          mm5 = out(end);
        M_est(j,5) = mm5;
        M_est(j,4) = mm4;
    end 
   
end

disp(M_est);

% write your helper functions below

function [d , dth] = get_odom(new_ticks,old_ticks)

    RadiusWheel = 0.0325;
    Numm = 370;
    WheelAxle = 0.32/2;
    
    minn1 = new_ticks;
    minn2 = old_ticks;
    dt = minn1 - minn2;
    
    odomTop = 2*pi*RadiusWheel*dt;
    odombot = Numm;
    dist = odomTop / odombot;
    
    ddmean = mean(dist);
    d = ddmean;
    left_value = dist(1);
    right_value = dist(2);
    
    dth11 = right_value;
    dth22 = left_value;
    dthbot = WheelAxle;
    dth = (dth11 - dth22) / dthbot;
end
function [mu,Sigma] = predict_step(mu,Sigma,d,dth,R)

    SizeOfSigma = size(Sigma);
    part1 = mu(1:3);
    inn1 = d*cos(mu(3));
    inn2 = d*sin(mu(3));
    inn3 = dth;
    part2 = [inn1; inn2 ;inn3];
    mu(1:3) = part1 + part2;
    
    zz = 0;
    onee = 1;
    col3 = -d*sin(mu(3));
    dcoss = d*cos(mu(3));
    
    JacobianX = [onee, zz, col3 ;
            zz, onee, dcoss ;
            zz ,zz ,onee];
    
        cosss = cos(mu(3));
        sinn = sin(mu(3));
    JacobianU = [cosss,  zz;
                    sinn, zz;
                    zz ,onee];
    sizeeeee = [3 , 3];
    if SizeOfSigma ~= sizeeeee
        Ipasss = eye(10);
        I = Ipasss;
        JPASS = blkdiag(JacobianX,I);
        JacobianX = JPASS;
        
        CC1 = JacobianU;
        CC2 = zeros(10,2);
        JacobianU = [CC1;CC2];
        SigmaPlus1 = JacobianX*Sigma*JacobianX';
        SigmaPlus2 = JacobianU*R*JacobianU';
        Sigma = SigmaPlus1 + SigmaPlus2;
    end
    SSigma1 = JacobianX*Sigma*JacobianX';
    SSigma2 = JacobianU*R*JacobianU';
    
    Sigma = SSigma1 + SSigma2;
 
end


function [mu, Sigma] = initLandmarks(z,Q,mu,Sigma)

    llpass = length(mu);
    LengthOfMU = llpass;
    
    for i = 1:length(z)
    
        mu3 = mu(3);
        ThetaVall = mu3;
        mu2pass = mu(2);
        YVall = mu2pass;
        mu1pass = mu(1);
        XVall = mu1pass;
        
        
    
        Range = z(i,1);
        Beearing = z(i,2);
        
        NUM3 = 3;
        if Range == 0 && LengthOfMU == NUM3
            NUMM2 = 2;
            NUM3 = 3;
            ZZERO = 0;
            mu(NUMM2 * i + NUMM2 : NUMM2 * i + NUM3) = ZZERO;
            Sigma(2*i+2:2*i+3,2*i+2:2*i+3) = ZZERO;
        
        elseif Range == 0 && LengthOfMU ~= 3
            NUMM2 = 2;
            NUM3 = 3;
            mu(  NUMM2* i + NUMM2 : NUMM2 * i + NUM3 ) = mu( NUMM2 * i  + NUMM2 : NUMM2 * i + NUM3);
        
        else
            NUMM2 = 2;
            NUM3 = 3;
            lnew = [XVall + Range * cos(ThetaVall + Beearing); 
                YVall + Range * sin(ThetaVall + Beearing)];
            mu(NUMM2 * i + NUMM2 : NUMM2 * i + NUM3) = lnew;

            LZZ1 = cos(ThetaVall + Beearing);
            LZZ2 = -Range*sin(ThetaVall + Beearing);
            LZZ3 = sin(ThetaVall + Beearing) ;
            LZZ4 = Range*cos(ThetaVall + Beearing);
            Lz = [ LZZ1 , LZZ2; 
                 LZZ3,LZZ4 ;];
            multipli1 = Lz*Q;
            multipli2 = multipli1 * Lz';
            Sigma(2*i+2:2*i+3,2*i+2:2*i+3) = multipli2;
        end 
    end
end 

function [mu, Sigma] = update_step(landmarkID,zi,Q,mu,Sigma)
    
    mmuu3 = mu(3);
    thetaVall = mmuu3;
    muu2 = mu(2);
    YVall = muu2;
    muu1 =  mu(1);
    XVall =muu1;

    innpass = landmarkID;
    i = innpass;
    
    ntwo = 2;
    nthree = 3;
    none = 1;
    ILenn = i * ntwo + nthree - none;
    
    muux = mu(ILenn);
    XCalcc = muux;
    
    muuy = mu(ILenn+1);
    YCalcc = muuy;
    
    root1 = (XVall-XCalcc)^2;
    root2 = (YVall-YCalcc)^2;
    
    RCalc = sqrt(root1 + root2);
    
    landtop = (length(mu)-3) ;
    NumLandmarkss = landtop / 2;

        
    Gmat1 = (XCalcc-XVall)/RCalc ;
    Gmat2 = (YCalcc-YVall)/RCalc;
    Gmat3 = -(YCalcc-YVall)/RCalc^2;
    Gmat4 = (XCalcc-XVall)/RCalc^2;
    GMatt = [ Gmat1,  Gmat2   ; Gmat3  , Gmat4];
    
    zerooarray = zeros(2,(i-1)*2);
    ZeroArrayHold = zerooarray;
    
    teemp = zeros(2,(NumLandmarkss-i)*2);
    ZeroArrayCheck = teemp ;
    
    GG1 = ZeroArrayHold;
    GG2 = GMatt;
    GG3 = ZeroArrayCheck;
    GMatt = [GG1,GG2, GG3];
    
    insidemat1 = -(XCalcc-XVall)/RCalc ;
    insidemat2 = -(YCalcc-YVall)/RCalc;
    insidemat3 = 0 ;
    insidemat4 = (YCalcc-YVall)/RCalc^2;
    insidemat5 = -(XCalcc-XVall)/RCalc^2 ;
    insidemat6 = -1;
    innsideGG = [ insidemat1, insidemat2 ,   insidemat3; insidemat4 , insidemat5 , insidemat6 ];
    GMatt = [innsideGG, GMatt];
    KKTop = (Sigma * GMatt');
    KKBot = (GMatt * Sigma * GMatt' + Q);
    K = KKTop/ KKBot;
   
    insidee1 = (XVall-XCalcc)^2;
    insidee2 = (YVall-YCalcc)^2;
    insidee3 = atan2(YCalcc-YVall, XCalcc-XVall);
    
    heightcal = [ sqrt(insidee1 + insidee2);
        insidee3 - thetaVall];
    
    heightcal(2) = wrapToPi(heightcal(2));

    zminus = zi';
    minused = heightcal;
    delta = zminus  - minused;
    
    delta(2) = wrapToPi(delta(2));

    parrrt1 = mu;
    parrrt2 = K * delta;
    mu = parrrt1 + parrrt2;
    
    ppp1 = (eye(size(Sigma)) - K * GMatt);
    ppp2 = Sigma;
    Sigma = ppp1 * ppp2 ;


end

function Z = sense(I)
     % Use the chromatacity instead
    img = double(I);
    imgTop = img(:,:,1) + img(:,:,2) ;
    imgTotal = imgTop + img(:,:,3);
    
    
    RedTop = img(:,:,1);
    GreenTop = img(:,:,2);
    BlueTop = img(:,:,3);
    
    redChannel = RedTop ./imgTotal;
    greenChannel = GreenTop ./imgTotal;
    blueChannel = BlueTop ./imgTotal;
    
    redd = redChannel;
    greenn = greenChannel;
    bluee = blueChannel;
    
    Point6 = 0.6;
    Point4 = 0.4;

    redBW = redd > Point6;
    blueBW = bluee > Point4;
    yellowBW = redd > Point4 & greenn > Point4;
    
    try
        rr1 = regionprops(redBW);
        redblobs = rr1;
        rr2 = cat(1,redblobs.Area);
        RedAreaas = rr2;
        rr3 = cat(1,redblobs.BoundingBox);
        BBoxRed = rr3;
        rr4 = cat(1,redblobs.Centroid);
        CentroidRed = rr4;
        [BoundsRed,~,~,~] =  bwboundaries(redBW);
        
        blueblobs = regionprops(blueBW);
        BlueAreaas = cat(1,blueblobs.Area);
        BBoxBlue = cat(1,blueblobs.BoundingBox);
        CentroidBlue = cat(1,blueblobs.Centroid);
        [BoundsBlue,~,~,~] = bwboundaries(blueBW);
        
        yellowblobs = regionprops(yellowBW);
        YellowAreaas = cat(1,yellowblobs.Area);
        BBoxYellow = cat(1,yellowblobs.BoundingBox);
        CentroidYellow = cat(1,yellowblobs.Centroid);
        [BoundsYellow,~,~,~] = bwboundaries(yellowBW);
    catch 
        Z = [];
        return
    end
    
    
    IFConditionPart1 = length(RedAreaas) < 1 || length(BlueAreaas) < 1;
    IFCondition =  IFConditionPart1 || length(YellowAreaas) < 1 ;
    if IFCondition
        Z = [];
        return
    end

    Z = [];
    
    RedNoise = find(RedAreaas < 30);
    RedAreaas(RedNoise) = [];
    BBoxRed(RedNoise,:) = [];
    CentroidRed(RedNoise,:) = [];
    BoundsRed(RedNoise) = [];
    
    BlueNoise = find(BlueAreaas < 30);
    BlueAreaas(BlueNoise) = [];
    BBoxBlue(BlueNoise,:) = [];
    CentroidBlue(BlueNoise,:) = [];
    BoundsBlue(BlueNoise) = [];
    
    YellowNoise = find(YellowAreaas < 30);
    YellowAreaas(YellowNoise) = [];
    BBoxYellow(YellowNoise,:) = [];
    CentroidYellow(YellowNoise,:) = [];
    BoundsYellow(YellowNoise) = [];
    
    BeaconID1 = 27;
    BeaconID2 = 30;
    BeaconID3 = 39;
    BeaconID4 = 45;
    BeaconID5 = 57;
    
    beacon_ids = [BeaconID1;
                  BeaconID2;
                  BeaconID3;
                  BeaconID4;
                  BeaconID5];
    
    Number1 = 1;
    Number2 = 2;
    for i = Number1:Number2
        
        [~,RedNoise] = max(RedAreaas);
        [~,BlueNoise] = max(BlueAreaas);
        [~,YellowNoise] = max(YellowAreaas);
        
        blobb1 = BBoxRed(RedNoise,2);
        blobb2 = BBoxBlue(BlueNoise,2);
        blobb3 = BBoxYellow(YellowNoise,2);
        blobs = [ blobb1 , blobb2 , blobb3 ];
        
        hee1 = BoundsRed(RedNoise);
        hee2 = BoundsBlue(BlueNoise);
        hee3 = BoundsYellow(YellowNoise);
        heights = [hee1 ;  hee2; hee3 ];
        
        cee1 = CentroidRed(RedNoise,:);
        cee2 = CentroidBlue(BlueNoise,:);
        cee3 = CentroidYellow(YellowNoise,:);
        centroids = [cee1 ;cee2  ; cee3];
        
        [~, passind] = sort(blobs);
        ind = passind;
        ind = fliplr(ind);
        
        if (isempty(ind) ~= 1)     
            String1 = '01';
            String2 = '10';
            String3 = '11';
            
            StringColl = {String1 String2 String3};
            
            BinaryALL = cell2mat(StringColl(ind));

            PassDeci = bin2dec(BinaryALL);
            TypeDeci = PassDeci;
            index = find(beacon_ids==TypeDeci);
            
            if (isempty(index) ~= 1)
                topval = heights(ind(1));
                Nancy = cell2mat(topval);
                bottomval = heights(ind(3));
                Donkey = cell2mat(bottomval);
                
                Number2 = 2;
                HeightPart1 = max(Nancy(:,1));
                HeightPart2 = min(Donkey(:,1));
                img_height = HeightPart1  - HeightPart2 + Number2;
                insidex = ind(2);
                XValImage = centroids( insidex , 1 );
                
                w_cam = 320;         
                h_cam = 240;
                focal_length = 3.6 * 10^-3;  
                AngleOfView = deg2rad(62.2);   
                HeightBeacon = 0.15;  
                h_sensor = 2.74 * 10^-3;

                
                RangeTop = HeightBeacon * focal_length * h_cam;
                RangeBot = (img_height * h_sensor) ;
                Range = RangeTop / RangeBot;

                
                angletemp_Top = (XValImage - (w_cam/2));
                angletemp_Bot = (w_cam/2);
                angle_temp = angletemp_Top / angletemp_Bot;
                bearingFront = -(AngleOfView/2);
                Bearing = bearingFront  * angle_temp;

                
                z = [Range; Bearing];
                RangeAndBearing = z;

                
                result = zeros(1,3);
                result(1) = TypeDeci;
                result(2) = RangeAndBearing(1);
                result(3) = RangeAndBearing(2);
                
                Z = [Z ; result];
            end 
        end 
        
        
       
        Coondition1 = length(RedAreaas) < 1 || length(BlueAreaas) < 1;
        CoonditionIF = Coondition1 || length(YellowAreaas) < 1;
        if CoonditionIF
            return
        end
              
    end 
   

end









