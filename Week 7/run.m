clear ; clc; close 

load map1r1.mat;
I = data(1).image;

ErodedImg = imopen(I , ones(4));

RGB = ErodedImg;

[BW,maskedRGBImage] = ForYellowColor(RGB);


s = regionprops(BW,'centroid');
Areas = regionprops(BW,'Area');

Areas.Area
centroids = cat(1,s.Centroid)

imshow(BW)
hold on
plot(centroids(:,1),centroids(:,2),'b*')
hold off

