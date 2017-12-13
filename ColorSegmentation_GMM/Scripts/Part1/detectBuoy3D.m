clear all
clc
folder = 'C:\Users\Steve\Documents\Grad School\Perception\Project 1\Buoy\UPDATED\Images\TrainingSet\Frames';
full_name= fullfile(folder, 'Frame28.jpg');  %Get frame 28 training frame


vid = VideoReader('detectbuoy.avi');
NewVid = VideoWriter('BuoyTest3D_SLOWER');
NewVid.FrameRate = 15;
open(NewVid);

for n = 1:60%video is only 5 frames per second, 200 frames long
frame = read(vid,n);

%GET CHANNELS
[L W asd] = size(frame);
redChannel = frame(:, :, 1);
greenChannel = frame(:, :, 2);
blueChannel = frame(:, :, 3);
Re1 = reshape(redChannel,L*W,1);
Ge1 = reshape(greenChannel,L*W,1);
Be1 = reshape(blueChannel,L*W,1);
Pixel = [Re1 Ge1 Be1];
Pixel = double(Pixel);%First entry is (1,1), second is 2nd row 1st col, third is 3rd row 1st col....

%Pixel = cell(L,W);
%Pixel is cell so every pixel in image has associated 1x3 [RGB] vector
% for b = 1:L
%     for c = 1:W
%         Pixel{b,c} = [double(redChannel(b,c)) double(greenChannel(b,c)) double(blueChannel(b,c))];
%     end
% end

[s1 s2] = size(redChannel);

%MEAN AND STND DEV
meanR = [ 243.6155  183.4058  126.6046 ];
covR =1.0e+03 .* [ 
    0.1642   -0.0859   -0.1085  ;
   -0.0859    1.6234    1.7266;
   -0.1085    1.7266    2.4514;
];

%yellow
meanY = [230.0208  235.0644  137.3284];
covY = 1.0e+03 .*[
    0.1073    0.0847    0.0213;
    0.0847    0.1707    0.1323;
    0.0213    0.1323    1.8013;
    ];

%green
meanG = [149.0179  211.3346  142.2255];
covG = 1.0e+03 .*[
  1.4567    0.7788    1.0762;
    0.7788    0.8441    0.4427;
    1.0762    0.4427    0.9414;];


%CREATE PROBABILITY MATRIX
Pr = mvnpdf(Pixel,meanR,covR);
Prmax = max(Pr(:));


%yellow Prob matrix
Py = mvnpdf(Pixel,meanY,covY);
Pymax = max(Py(:));

%green PRob matrix
Pg = mvnpdf(Pixel,meanG,covG);
Pgmax = max(Pg(:));


%CREATE MAPPED PROB MATRIX
input_range = Prmax - 0;% input_end - input_start;
inputY = Pymax-0;
inputG = Pgmax-0;
output_range =  255 - 0; %output_end - output_start;

MapR = Pr*(output_range/(input_range));
MapY = Py*(output_range/(inputY));
MapG = Pg*(output_range/(inputG));

%Reshape back to shape of image
MapR = reshape(MapR,L,W);
MapY = reshape(MapY,L,W);
MapG = reshape(MapG,L,W);

%We mapped so full probability is 255. Turn all pixels with under 250 value
%to black, should only be left with red buoy. Make a new image with blob
%near buoy.
MapRnew1 = MapR;
indexR = (MapRnew1 < 250);
MapRnew1(indexR) = 0;

se1 = strel('disk', 10);
MapRnew = imdilate(MapRnew1,se1);
imshow(MapRnew)

%mapped yellow
MapYnew = MapY;
indexY = (MapYnew < 230);
indexYr = (MapR > 22);
MapYnew(indexY) = 0;
MapYnew(indexYr) = 0;

se1 = strel('disk', 10);
MapYnew2 = imdilate(MapYnew,se1);

%mapped green
MapGnew = MapG;
indexG = (MapGnew < 240);
MapGnew(indexG) = 0;%After this have some pixels on each buoy
indexGy = (MapYnew > 250);

        

se1 = strel('disk', 10);
MapGnew2 = imdilate(MapGnew,se1);
imshow(MapGnew)
%figure


%COUTNOUR PREPARATION
Rcheck = 0;
Ycheck = 0;
Gcheck =0;

%Draw line on buoy from MapR which looks a lot nicer
MapRnew = im2bw(MapRnew);
BlobR = regionprops(MapRnew,'Centroid');
if (length(BlobR) > 0)
rx = BlobR(1).Centroid(1);%Find center of blob we made using pixel. Blob isnt perfect representation of Buoy
ry = BlobR(1).Centroid(2);

xmin = rx - 75;%Define the nearby area of our Blob
xmax = rx + 75;
ymin = ry - 75;
ymax = ry + 75;
RCopy = MapR;%Create an image that only keeps the white in the nearby area. So use image MapR that has good looking buoy outline, keep white only on bouy in that image
for i = 1:s1
    for j = 1:s2
        if (j<xmin || j>xmax || i<ymin || i>ymax)
            RCopy(i,j) = 0;
        end
    end
end
Rcheck = 1;


end

%Yellow countour prep
MapYnew = im2bw(MapYnew);
BlobY = regionprops(MapYnew,'Centroid');
if (length(BlobY) > 0)
yx = BlobY(1).Centroid(1);%Find center of blob we made using pixel. Blob isnt perfect representation of Buoy
yy = BlobY(1).Centroid(2);

xmin = yx - 75;%Define the nearby area of our Blob
xmax = yx + 75;
ymin = yy - 75;
ymax = yy + 75;
YCopy = MapY;%Create an image that only keeps the white in the nearby area. So use image MapR that has good looking buoy outline, keep white only on bouy in that image
for i = 1:s1
    for j = 1:s2
        if (j<xmin || j>xmax || i<ymin || i>ymax)
            YCopy(i,j) = 0;
        end
    end
end
Ycheck = 1;
end

%green countour prep
MapGnew = im2bw(MapGnew);
BlobG = regionprops(MapGnew,'Centroid');
if (length(BlobG) > 0)
 for i = 1:length(BlobG)
if (abs(BlobG(i).Centroid(1)-yx) > 70)
gx = BlobG(i).Centroid(1);%Find center of blob we made using pixel. Blob isnt perfect representation of Buoy
gy = BlobG(i).Centroid(2);
end
 end
xmin = gx - 40;%Define the nearby area of our Blob
xmax = gx + 40;
ymin = gy - 40;
ymax = gy + 40;
GCopy = MapG;%Create an image that only keeps the white in the nearby area. So use image MapR that has good looking buoy outline, keep white only on bouy in that image
index = (MapG<90);
GCopy(index) = 0;

% GCopy = bwareaopen(GCopy,20);
% seG = strel('disk',5);
% GCopy = imdilate(GCopy,seG);
% GCopy = imfill(GCopy,'holes');

for i = 1:s1
    for j = 1:s2
        if (j<xmin || j>xmax || i<ymin || i>ymax)
            GCopy(i,j) = 0;
        end
    end
end
Gcheck = 1;
end

imshow(frame)
hold on
abs(ry-yy)
%DRAW COUNTOURS ON BUOY
if (Rcheck == 1)
RCopy = im2bw(RCopy);%Need to be binary for bw boundaries
B = bwboundaries(RCopy);%Boundaries of the white in binary image
for  k =1:length(B)
boundary = B{k};
if (abs(rx-yx) > 70 && abs(ry-yy) < 100)
plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 3)%Plot the boundaries in red
end
end 
end

%draw contours for yellow
if (Ycheck == 1)
YCopy = im2bw(YCopy);%Need to be binary for bw boundaries
B = bwboundaries(YCopy);%Boundaries of the white in binary image
for  k =1:length(B)
boundary = B{k};
plot(boundary(:,2), boundary(:,1), 'y', 'LineWidth', 3)%Plot the boundaries in yellow
end 
end

%draw coutours for green\
if (Gcheck == 1)
GCopyw = im2bw(GCopy);%Need to be binary for bw boundaries
GCopy = bwareaopen(GCopy,5);
GCopy = imdilate(GCopy,strel('disk',2));
GCopy = imfill(GCopy, 'holes');
Gcen = regionprops(GCopy,'Centroid');
if (length(Gcen) > 0)
gx = Gcen(1).Centroid(1);
gy = Gcen(1).Centroid(2);
scatter(gx,gy,400,'g','LineWidth',3);
end
% B = bwboundaries(MapGnew2);%Boundaries of the white in binary image
% B = bwboundaries(GCopy);
% for  k =1:length(B)
% boundary = B{k};
% if (((yy-gy)^2+(yx-gx)^2)^.5 > 70)
% plot(boundary(:,2), boundary(:,1), 'g', 'LineWidth', 2)%Plot the boundaries in red
% end
% end 
end

title(n)
f = getframe(gca);
im = frame2im(f);
writeVideo(NewVid,im) %Add frame to video
set(gcf,'visible','off')
cla%clear axes, this makes it so it doesnt slow down over time by stacking images
end
close(NewVid);

%Works but takes a long time. Create matrix Prob that contains probability
%value for every pixel in redChannel. mean and std of red buoy are used,
%every pixel value has a probability from 0 to 1
% for i = 1:s1%go through every pixel in red channel
%     for j = 1:s2
%         val = redChannel(i,j);%get red Channel value, 0-255
%         p = normcdf(double(val),u,std);%find probability that this value is the red buoy
%         Prob(i,j) = p;%populate Prob matrix with probability for each pixel
%         [i j n]%watch how long it takes to run
%     end
% end

%CREATE BINARY
%Using indexing much faster than for loops
%red
% BW = im2bw(frame);%create binary image of frame
% threshold = .99;%minimum probability to be considered a buoy
% %indexW is all places where Prob greather than threshold, want these pixels
% %white. indexB is where less than threshold, want these black
% indexW = (Pr >= threshold);
% indexB = (Pr < threshold);
% BW(indexW) = 1;
% BW(indexB) = 0;
% 
% 
% for i =1:480
%     for o = 1:640
%         if (Pr(i,o) > threshold && Pg(i,o) < .01 && Py(i,o) <1.000000000000000e-25)
%             BW(i,o) = 1;
%         else
%             BW(i,o) = 0;
%         end
%     end
% end
% imshow(BW)
% figure
% 
% 
% %yellow
% BWY = im2bw(frame);
% thresholdY = .99; %.75;
% % indexWY = (Py >= thresholdY);
% % indexBY = (ProbY < thresholdY);
% % BWY(indexWY) = 1;
% % BWY(indexBY) = 0;
% % BWY(indexW)  = 0;
% 
% for i =1:480
%     for o = 1:640
%         if (Py(i,o) > thresholdY && Pr(i,o) > .5 && Pg(i,o) < .2)
%             BWY(i,o) = 1;
%         else
%             BWY(i,o) = 0;
%         end
%     end
% end
% 
% 
% %green
%  BWG = im2bw(frame,.75);
%  H = BWG;
% 
% thresholdG = .999;
% indexWG = (Pg > thresholdG);
% indexBG = (Pg < thresholdG);
% BWG(indexWG) = 1;
% BWG(indexBG) = 0;
% 
% 
% 
% for i =1:480
%     for o = 1:640
%         if (Pg(i,o) > thresholdG && Pr(i,o) < .00000000000000000000000000000001 && Py(i,o)<.6)
%             BWG(i,o) = 1;
%             [i o]
%             
%         else
%             BWG(i,o) = 0;
%         end
%     end
% end
% 
% 
% 
% 
% %Def worksbut takes long, Creates binary image based on threshold
% %propbability and Prob matrix
% % for m = 1:s1%check every value of Prob
% %     for n = 1:s2
% %         if (Prob(m,n)>= threshold)
% %             BW(m,n) = 1;%make white if probability it is buoy is greater than threshold
% %         else
% %             BW(m,n) = 0;%make black if probability it is buoy is less than threshold
% %         end
% %         [m n 2]%Watch how long it takes
% %     end
% % end
% 
% %CLEAN UP BINARY IMAGE
% %Look at original frame and binarized frame
% %red
% %BW2 = bwareaopen(BW,2);%get rid of little white specs
% se1 = strel('disk', 6);%make binary buoy a little more defined
% BW3 = imdilate(BW,se1);
% imshow(BW3)
% 
% %yellow
% %BWY2 = bwareaopen(BWY,50);
% se2 = strel('disk',8);
% BWY3 = imdilate(BWY,se2);
% imshow(BWY3)
% 
% %green
% %BW2G = bwareaopen(BWG,10);
% 
% %We will prob get yellow buoy here so compare redChannel at locations
% %found. more red means not green buoy
% %indy = (redChannel > 170);
% %BW2G(indy) = 0;
% se3 = strel('disk', 6);
% BW3G = imdilate(BWG,se3);
% %imshow(BW3G)
% %figure
% 
% %imshow(BW3)
% %figure
% %imshow(BWY3)
% %figure
% %imshow(BW3G)
% %figure
% imshow(frame)
% 
% hold on
% 
% %TRY REGIONPROPS TO CIRCLE BUOYS
% %red
% measurements = regionprops(BW3, 'BoundingBox', 'Centroid');
% if (size(measurements) > 0)
% rx = measurements(1).Centroid(1);%circle center is centroid of blob
% ry = measurements(1).Centroid(2);
% rcircSize = measurements(1).BoundingBox(3)*measurements(1).BoundingBox(4);%circle area is area of box
% %scatter(rx,ry,rcircSize,'r')%plot circle
% end
% %DRAW CONTOURS
% %Draw Contours around found buoy. Show frame, then draw border of white
% %blob in binary image
% %figure
% %red
% %Try to draw countours on edges of blob
% B = bwboundaries(BW3);
% for  k =1:length(B)
% boundary = B{k};
% plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2)
% title(n)
% end 
% 
% %yellow
% %Measurements uses regionprops, finds centroid and area of box, draws
% %circle with scatter at centroid with same area
% measurementsY = regionprops(BWY3, 'BoundingBox', 'Centroid');
% if (size(measurementsY) > 0)
% yx = measurementsY(1).Centroid(1);%circle center is centroid of blob
% yy = measurementsY(1).Centroid(2);
% ycircSize = measurementsY(1).BoundingBox(3)*measurementsY(1).BoundingBox(4);%circle area is area of box
% %scatter(yx,yy,ycircSize,'y')%plot circle
% end
% hold on
% %Try to draw countours on edges of blob
% BY = bwboundaries(BWY3);
% for  k =1:length(BY)
% boundary = BY{k};
% plot(boundary(:,2), boundary(:,1), 'y', 'LineWidth', 2)
% title(n)
% end 
% 
% %green
% measurementsG = regionprops(BW3G, 'BoundingBox', 'Centroid');
% if (size(measurementsG) > 0)
% gx = measurementsG(1).Centroid(1);%circle center is centroid of blob
% gy = measurementsG(1).Centroid(2);
% gcircSize = measurementsG(1).BoundingBox(3)*measurementsG(1).BoundingBox(4);%circle area is area of box
% %scatter(gx,gy,gcircSize,'g')%plot circle
% end
% hold on
% %Try to draw countours on edges of blob
% BG = bwboundaries(BW3G);
% for  k =1:length(BG)
% boundary = BG{k};
% plot(boundary(:,2), boundary(:,1), 'g', 'LineWidth', 2)
% title(n)
% end 
% 
% 
%  
% %NORMCDF REFERENCE
% %value, mean, std --> returns probability of value
% %normcdf(255,u,std)
% 
% %Plot gaussian for red buoy
% % x = [0:10:400];
% % norm = normpdf(x,u,std);
% % plot(x,norm,'r')
% 
% 
% f = getframe(gca);
% im = frame2im(f);
% writeVideo(NewVid,im) %Add frame to video
% %cla%clear axes, this makes it so it doesnt slow down over time by stacking images
% end
% close(NewVid);
%     
% set (gcf, 'WindowButtonMotionFcn', @mouseMove);%X and y are mixed up I think
