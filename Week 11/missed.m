function Z = sense(I)
    % Using the inputted image
    % CHECK Color
    ImageNew = double(I);
    Part1 = ImageNew(:,:,1) + ImageNew(:,:,2);
    ChannelTotal = Part1 + ImageNew(:,:,3);
    
    TopRed   = ImageNew(:,:,1);
    TopGreen = ImageNew(:,:,2);
    TopBlue  = ImageNew(:,:,3);
    
    ChannelRed   = TopRed ./ ChannelTotal ;
    ChannelGreen = TopGreen ./ ChannelTotal ;
    ChannelBlue  = TopBlue ./ ChannelTotal ;
    
    RedValue = 0.6;
    BlueValue = 0.4;
    
    BWMaskRed = ChannelRed > RedValue;
    BWMaskBlue = ChannelBlue > BlueValue;
    BWMaskYellow = ChannelRed > BlueValue & ChannelGreen > BlueValue;
    
    try
        BLOBSCRed = regionprops(BWMaskRed);
        AreaRed = cat(1,BLOBSCRed.Area);
        BBoxRed = cat(1,BLOBSCRed.BoundingBox);
        CentroidRed = cat(1,BLOBSCRed.Centroid);
        [BoundRed,~,~,~] =  bwboundaries(BWMaskRed);
        
        BLOBSCBlue = regionprops(BWMaskBlue);
        AreaBlue = cat(1,BLOBSCBlue.Area);
        BBoxBlue = cat(1,BLOBSCBlue.BoundingBox);
        CentroidBlue = cat(1,BLOBSCBlue.Centroid);
        [BoundBlue,~,~,~] = bwboundaries(BWMaskBlue);
        
        BLOBSCYellow = regionprops(BWMaskYellow);
        AreaYellow = cat(1,BLOBSCYellow.Area);
        BBoxYellow = cat(1,BLOBSCYellow.BoundingBox);
        CentroidYellow = cat(1,BLOBSCYellow.Centroid);
        [BoundYellow,~,~,~] = bwboundaries(BWMaskYellow);
    catch
        
        Z = [];
        return
    end
    
    Part1CFI = length(AreaRed) < 1 ||  length(AreaBlue) < 1;
    ConditionForIf = Part1CFI || length(AreaYellow) < 1;
    if ConditionForIf
        Z = [];
        return
    end
    
    Z = [];
    
    NumThirty = 30;
    
    NoiseInRed = find(AreaRed < NumThirty);
    AreaRed(NoiseInRed) = [];
    BBoxRed(NoiseInRed,:) = [];
    CentroidRed(NoiseInRed,:) = [];
    BoundRed(NoiseInRed) = [];
    
    NoiseInBlue = find(AreaBlue < NumThirty);
    AreaBlue(NoiseInBlue) = [];
    BBoxBlue(NoiseInBlue,:) = [];
    CentroidBlue(NoiseInBlue,:) = [];
    BoundBlue(NoiseInBlue) = [];
    
    NoiseInYellow = find(AreaYellow < NumThirty);
    AreaYellow(NoiseInYellow) = [];
    BBoxYellow(NoiseInYellow,:) = [];
    CentroidYellow(NoiseInYellow,:) = [];
    BoundYellow(NoiseInYellow) = [];
    
    
    Beacon1 = 27;
    Beacon2 = 30;
    Beacon3 = 39;
    Beacon4 = 45;
    Beacon5 = 57;
    
    BeaconsID = [Beacon1;
          Beacon2;
          Beacon3;
          Beacon4; 
          Beacon5];
    
     onee = 1;
     twoo = 2;
     for ind = onee : twoo 
        [~,NoiseInYellow] = max(AreaYellow);
        [~,NoiseInBlue] = max(AreaBlue);
        [~,NoiseInRed] = max(AreaRed);
        
        GH1 = BoundRed(NoiseInRed);
        GH2 = BoundBlue(NoiseInBlue);
        GH3 = BoundYellow(NoiseInYellow);
        GetHeights = [GH1 ; GH2 ; GH3];
        
        Cee1 = CentroidRed(NoiseInRed,:) ;
        Cee2 = CentroidBlue(NoiseInBlue,:);
        Cee3 = CentroidYellow(NoiseInYellow,:);
        CENTROIDS = [ Cee1 ; Cee2 ; Cee3 ];
        
        b1 = BBoxRed(NoiseInRed,2);
        b2 = BBoxBlue(NoiseInBlue,2);
        b3 = BBoxYellow(NoiseInYellow,2);
        BLOBS = [b1 , b2 , b3];
        
        [~, PASSIN] = sort(BLOBS);
        Innd = PASSIN;
        Innd = fliplr(Innd);
        
        if (isempty(Innd) ~= 1)
            Stringg1 = '01';
            Stringg2 = '10';
            Stringg3 = '11';
            
            BeaconColorStr = {Stringg1,Stringg2,Stringg3};
            BinaryPass = cell2mat(BeaconColorStr(Innd));
            BinaryStr = BinaryPass;
            BinaryStr = bin2dec(BinaryStr);
            
            
            indexingg = find(BeaconsID == BinaryStr);
            
            if (isempty(indexingg) ~= 1)
                
                NNPass = cell2mat(GetHeights(ind(1)));
                NancyEq = NNPass;
                DDDPass = cell2mat(GetHeights(ind(3)));
                DonkeyEq = DDDPass;
                
                HMM1 = max(NancyEq(:,1));
                HMM2 = min(DonkeyEq(:,1));
                Twoo = 2;
                HeightIMM = HMM1 - HMM2 + Twoo;
                
                
                XposIMG = CENTROIDS(Innd(2),1);
                % Get range and Bearing CHECK
                TwoForty = 240;
                CameraHeight = TwoForty;
                ThreeTwenty = 320;
                CameraWidth = ThreeTwenty;         
                
                ThreePSix = 3.6;
                focal = ThreePSix * 10^-3; 
                
                sixtytwo = 62.2;
                AngleView = deg2rad(sixtytwo);
                
                POneFive = 0.15;
                BeaconHeightt = POneFive; 
                TwoSeven = 2.74;
                SensorHeight = TwoSeven * 10^-3;
                 
                

                % range
                NancyEQQQ = BeaconHeightt * focal * CameraHeight;
                DonkeyEQQ = (HeightIMM * SensorHeight);
                Range = NancyEQQQ / DonkeyEQQ ;

                % bearing
                AngleeeTop = (XposIMG - (CameraWidth/2)) ;
                Donkey = (CameraWidth/2);
                TempAnglee =  AngleeeTop / Donkey;
                Bearing = -(AngleView/2) * TempAnglee;


                RangeANDBearing = [Range; Bearing];
                
                FinalRes = zeros(1,3);
                
                FinalRes(1) = bin2dec(BinaryStr);
                
                FinalRes(2) = RangeANDBearing(1);
                
                FinalRes(3) = RangeANDBearing(2);
                
                Z = [Z ; FinalRes];
            end 
        end
        IFConditionPart1 = length(AreaRed) < 1 || length(AreaYellow) < 1;
        Conditionnnn = IFConditionPart1 || length(AreaYellow) < 1 ; 
        if  Conditionnnn
            return
        end
        
        
     end
end
