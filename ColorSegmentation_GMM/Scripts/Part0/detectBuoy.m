clear all
clc

% two finger scroll disable
% !synclient HorizEdgeScroll=0 HorizTwoFingerScroll=0 

vid = VideoReader('detectbuoy.avi');
NewVid = VideoWriter('TEST');
NewVid.FrameRate = 15;
open(NewVid);

for n = 1:60 %200 frames at 5 frames per secondi
frame = read(vid,n);
[L W e] = size(frame);

%GET CHANNELS
redChannel = frame(:, :, 1);
greenChannel = frame(:, :, 2);
blueChannel = frame(:, :, 3);
yellowChannel = (double(redChannel)+double(greenChannel))/2;

%MEAN AND STND DEV
%calculated mean and std dev values for red buoy
meanR = 243.5844;
stdR = 12.8897;

%mean and std dev for yellow buoy
meanY = 232.5877;
stdY = 10.5249;

%mean and std dev for green buoy
meanG = 211.4465;
stdG = 28.9925;

%CREATE PROBABILITY MATRIX
%probability that a given pixel is a pixel on the red buoy
P = normpdf(double(redChannel),meanR,stdR);
Pmax = max(P(:));%Find the max probability

%yellow
Py = normpdf(double(yellowChannel),meanY,stdY);
Pymax = max(Py(:));

%green
greenChannel = (double(greenChannel)+double(blueChannel))/2;
Pg = normpdf(double(greenChannel),meanG,stdG);
Pgmax = max(Pg(:));


%MAPPED PROBABILITY MATRIX
%Map values from probability matrix so that 255 means full probability and
%0 means well zero probability
input_range = Pmax - 0;% input_end - input_start;
Inputy = Pymax;%yellow 
Inputg = Pgmax;
output_range =  255 - 0; %output_end - output_start;

%mapped probability matrices
MappedR = P * (output_range/(input_range));%mapped Probability matrix for red, max of 255 min of 0
MappedY = Py * (output_range/Inputy);%mapped prob for yellow
MappedG = Pg * (output_range/Inputg);%mapped prob green



%CREATE BINARY
%Find red
thresholdR = 240;%threshold probaility value for being a red buoy, must be greater than this
RedBin = im2bw(frame);%%if red over 240, less than 50 on yellow, then it is a red buoy. 255 is max, 0 is bottom of how sure we are
for i =1:480%go though every pixel
    for o = 1:640
        if (MappedR(i,o) > thresholdR && MappedY(i,o)<50)%if current pixel's redChannel value has red buoy probability over threshold and probability under yellow buoy
            RedBin(i,o) = 1;%Make Binary image red. So if (probably a red buoy and probabily not a yellow buoy)
        else
            RedBin(i,o) = 0;%If not red buoy make it black
        end
    end
end
RedBin2 = bwareaopen(RedBin,6);%Get rid of pixels here and there that made the cut
se1 = strel('disk', 10);%make binary buoy a little more defined
RedBin3 = imdilate(RedBin2,se1);%more defined

%Find yellow, same method
thresholdY = 240;
YellowBin = im2bw(frame);
for i =1:480
    for o = 1:640
        if (MappedY(i,o) > thresholdY  && MappedR(i,o)< 199 && MappedR(i,o) > 150 && MappedG(i,o) < 90)% && MappedG(i,o)< 187)
            YellowBin(i,o) = 1;
        else
            YellowBin(i,o) = 0;
        end
    end
end
YellowBin2 = bwareaopen(YellowBin,1);%was 2
YellowBin3 = imdilate(YellowBin2,se1);


%Find green, same method
thresholdG = 250;
GreenBin   = im2bw(frame);
for i =1:480
    for o = 1:640
        if (MappedG(i,o) > thresholdG && MappedR(i,o)<50 && MappedY(i,o) > 100)
            GreenBin(i,o) = 1;
        else
            GreenBin(i,o) = 0;
        end
    end
end 
GreenBin2 = bwareaopen(GreenBin,4);
GreenBin3 = imdilate(GreenBin2,se1);
imshow(GreenBin);
% figure

%COUTNOUR PREPARATION
Rcheck = 0;
Ycheck = 0;
Gcheck =0;

%Draw line on buoy from MapR which looks a lot nicer
BlobR = regionprops(RedBin3,'Centroid');
if (length(BlobR) > 0)
rx = BlobR(1).Centroid(1);%Find center of blob we made using RedBin3. Blob isnt perfect representation of Buoy
ry = BlobR(1).Centroid(2);

xmin = rx - 75;%Define the nearby area of our Blob
xmax = rx + 75;
ymin = ry - 75;
ymax = ry + 75;
RCopy = MappedR;%Create an image that only keeps the white in the nearby area. So use image MapR that has good looking buoy outline, keep white only on bouy in that image
for i = 1:L
    for j = 1:W
        if (j<xmin || j>xmax || i<ymin || i>ymax)
            RCopy(i,j) = 0;
        end
    end
end
Rcheck = 1;
end


%Yellow countour prep
BlobY = regionprops(YellowBin3,'Centroid');
if (length(BlobY) > 0)
yx = BlobY(1).Centroid(1);%Find center of blob we made using pixel. Blob isnt perfect representation of Buoy
yy = BlobY(1).Centroid(2);

xmin = yx - 75;%Define the nearby area of our Blob
xmax = yx + 75;
ymin = yy - 75;
ymax = yy + 75;
YCopy = MappedY;%Create an image that only keeps the white in the nearby area. So use image MapR that has good looking buoy outline, keep white only on bouy in that image
for i = 1:L
    for j = 1:W
        if (j<xmin || j>xmax || i<ymin || i>ymax)
            YCopy(i,j) = 0;
        end
    end
end
Ycheck = 1;
end

%Green Prep
BlobG = regionprops(GreenBin2,'Centroid','Area');
if (length(BlobG) > 0)
%Look at where every blob is in GreenBin2, if they are like right on top of
%yellow then assign gy = yy, gx = yx, so that when drawing countours the
%green isnt drawn. There is a condition down there which checks distance
for s = 1:length(BlobG)
gx = BlobG(s).Centroid(1);%Find center of blob we made using pixel. Blob isnt perfect representation of Buoy
gy = BlobG(s).Centroid(2);
if (Ycheck == 1 && (((yy-gy)^2+(yx-gx)^2)^.5 <70 || BlobG(s).Area > 100))%If we have info for y, and we see green data near yellow, or huuggeee green data
    gy = yy;
    gx = yx;
    break
end
end
end

imshow(frame)
hold on

%DRAW CONTOURS
%Draw red
%Find general location of buoy
if(Rcheck == 1)
RCopy = im2bw(RCopy);
B = bwboundaries(RCopy);%Boundaries of the white in binary image
for  k =1:length(B)
boundary = B{k};
plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2)%Plot the boundaries in red
end 
end
        



%Draw yellow
if(Ycheck == 1)
YCopy = im2bw(YCopy);
B = bwboundaries(YCopy);%Boundaries of the white in binary image
for  k =1:length(B)
boundary = B{k};
plot(boundary(:,2), boundary(:,1), 'y', 'LineWidth', 3)%Plot the boundaries in red
end 
end

% %Draw green
Gcen = regionprops(GreenBin3,'Centroid');
if (length(Gcen) > 0)
for i = 1:length(Gcen)
 if (abs(Gcen(i).Centroid(1)-yx)> 70 && abs(Gcen(i).Centroid(2)-yy) < 70)
gx = Gcen(i).Centroid(1);
gy = Gcen(i).Centroid(2);
scatter(gx,gy,500,'g','LineWidth',3)
end
end
end
% B = bwboundaries(GreenBin3);%Boundaries of the white in binary image
% for  k =1:length(B)
% boundary = B{k};
% if (Ycheck == 1 && (((yy-gy)^2+(yx-gx)^2)^.5 > 70))
% plot(boundary(:,2), boundary(:,1), 'g', 'LineWidth', 2)%Plot the boundaries in red
% end 
% end



title(n)
f = getframe(gca);
im = frame2im(f);
writeVideo(NewVid,im) %Add frame to video
cla%clear axes, this makes it so it doesnt slow down over time by stacking images
end
close(NewVid);


