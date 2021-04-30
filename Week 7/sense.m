function Z = sense()
    % for week 7 task, detect IDs only
    % input: I is an RGB image of the robot's view
    % output: Z = 1xn where n is the number of beacons in the image.

    %for week 8 task, detect IDs and estimate range and bearing
    % input: I is an RGB image of the robot's view
    % output: Z = nx3 where Z(i,:) = [id range bearing] (bearing should be in degrees)
    
    clear ; clc; close 
    % Getting the image
    load map1r1.mat;
    I = data(1).image;
    
    RGB = I;
    
    %% For Blue mask
    [BWB,maskedRGBImage] = ForBlueColor(RGB);
    
    
    s_Blue = regionprops(BWB,'centroid');
    Areas_Blue = regionprops(BWB,'Area');
    
    Areas_Blue.Area
    centroids_Blue = cat(1,s_Blue.Centroid)

    %% For Red mask
    [BWR,maskedRGBImage] = ForRedColor(RGB);
    
    
    s_Red = regionprops(BWR,'centroid');
    Areas_Red = regionprops(BWR,'Area');
    
    Areas_Red.Area
    centroids_Red = cat(1,s_Red.Centroid)
    
  
    %% For yellow mask
    [BWY,maskedRGBImage] = ForYellowColor(RGB);
    
    
    s_Yellow = regionprops(BWY,'centroid');
    Areas_Yellow = regionprops(BWY,'Area');
    
    Areas_Yellow.Area
    centroids_Yellow = cat(1,s_Yellow.Centroid)
    
    figure;
    imshow(BWB+BWR+BWY)
    hold on
    plot(centroids_Blue(:,1),centroids_Blue(:,2),'b*')
    hold on
    plot(centroids_Red(:,1),centroids_Red(:,2),'b*')
    hold on
    plot(centroids_Yellow(:,1),centroids_Yellow(:,2),'b*')
    
 

end
    % write all your helper functions below and call them inside the function sense
