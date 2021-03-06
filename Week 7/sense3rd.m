function Z = sense3rd(I)
% input: I is an RGB image of the robot's view
% output: Z = 1xn where n is the number of beacons in the image.


% input: I is an RGB image of the robot's view
% output: Z = 1xn where n is the number of beacons in the image.

% for week 7 task, detect IDs only
% input: I is an RGB image of the robot's view
% output: Z = 1xn where n is the number of beacons in the image.

% input: I is an RGB image of the robot's view
% output: Z = 1xn where n is the number of beacons in the image.

% for week 7 task, detect IDs only
% input: I is an RGB image of the robot's view
% output: Z = 1xn where n is the number of beacons in the image.

%for week 8 task, detect IDs and estimate range and bearing
% input: I is an RGB image of the robot's view
% output: Z = nx3 where Z(i,:) = [id range bearing] (bearing should be in degrees)


    RGB = I;

    % For Blue mask
    [BWB,maskedRGBImage] = ForBlueColor2(RGB);
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
%     idsAllY = idsAllY';
    

    
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
    Z;
     
    
    % write all your helper functions below and call them inside the function sense
% end



    
    
    
    % write all your helper functions below and call them inside the function sense
end


function [BWB,maskedRGBImage] = ForBlueColor2(RGB)
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
channel1Min = 0.548;
channel1Max = 0.709;

% Define thresholds for channel 2 based on histogram settings
channel2Min = 0.507;
channel2Max = 0.947;

% Define thresholds for channel 3 based on histogram settings
channel3Min = 0.096;
channel3Max = 0.936;

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
% write all your helper functions below and call them inside the function sense
