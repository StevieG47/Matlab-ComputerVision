
clc

vid = VideoReader('detectbuoy.avi');
NewVid = VideoWriter('GMM_NEW_SLOWER');
NewVid.FrameRate = 15;
open(NewVid);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% GMMLEARNING.M %%%%%%%%%%%%%%%%%%%%%%%%%%%
%Get GMM infro



for n = 1:60%video is only 5 frames per second, 200 frames long
frame = read(vid,n);


%GET CHANNELS
[L W nf] = size(frame);
redChannel = frame(:, :, 1);
greenChannel = frame(:, :, 2);
blueChannel = frame(:, :, 3);


%Size of a color Channel
[s1 s2] = size(redChannel);

%CREATE PROBABILTIY MAP
%Make each channel a single column, create Data matrix with n rows
%observations, 3 columns dimensions to be used with gmm pdf
Re1 = reshape(redChannel,L*W,1);
Ge1 = reshape(greenChannel,L*W,1);
Be1 = reshape(blueChannel,L*W,1);
Pixel = [Re1 Ge1 Be1];
Pixel = double(Pixel);%First entry is (1,1), second is 2nd row 1st col, third is 3rd row 1st col....

%Probability Maps
MapR = pdf(Gmm,Pixel);%Crate probability for every pixel. But every pixel is not aligned in matrix they are in a list.
maxR = max(MapR(:));

MapY = pdf(GmmY,Pixel);
maxY = max(MapY(:));

MapG = pdf(GmmG,Pixel);
maxG = max(MapG(:));

%Reshape so it is in image format, (1,1) is first row first column
MapR = reshape(MapR,L,W);
MapY = reshape(MapY,L,W);
MapG = reshape(MapG,L,W);

%Change values of probailtiy map to 0-255
input_range = maxR - 0;% input_end - input_start;
inputY = maxY;
inputG = maxG;
output_range =  255 - 0; %output_end - output_start;

RRR = MapR*(output_range/(input_range));%multiply matrix by a quotient of ranges to map 0-->0 , max --> 255
YYY = MapY*(output_range/(inputY));
GGG = MapG*(output_range/(inputG));
%%%RRR is now mapped probability matrix for the red buoy, 0 means not red
%%%buoy, higher value with max of 255 means better chance of pixel belonging to a red buoy

%THRESHOLDING
Yim = YYY;%Copy of 1st probability map
indexY = (YYY < 250);%if prob under 250 (255 is max) then set pixel to 0
Yim(indexY) = 0;%set pixel to 0

%Red buoy image threshold
Rim = RRR;
indexR = (RRR < 230);
Rim(indexR) = 0;


Gim = GGG;
indexG = (GGG < 1);
Gim(indexG) = 0;
Gim2 = bwareaopen(Gim,60);



%Beef up the image
%MAYBEEE USE IMDILATE ON RRR AFTER REMOVING UNWANTED STUFF FROM THERE
%Rfin HAS LIKE NO PIXELS BUT RRR HAS LIKE THE WHOLE BUOY
se1 = strel('disk', 5);
se2 = strel('disk', 2);
se3 = strel('disk',1);
Rfin = imdilate(Rim,se1);
Yfin = imdilate(Yim,se1);%Beef up the couple of pixels that made the cut
Gfin = imdilate(Gim2,se1);


%Find coord where pixel/s remain after strict threshold
Rfin = im2bw(Rfin);
RegionR = regionprops(Rfin,'Centroid','Area');%Region of dilated pixel/couple of pixels that made the cut


if (length(RegionR) > 0)
rx = RegionR(1).Centroid(1);
ry = RegionR(1).Centroid(2);
end

Yfin = im2bw(Yfin);
RegionY = regionprops(Yfin,'Centroid');
if(length(RegionY) > 0)
yx = RegionY(1).Centroid(1);
yy = RegionY(1).Centroid(2);%get x y value of pixel that made cut. We know this is the area to look for th buoy
end

%Gfin = im2bw(Gfin);
RegionG = regionprops(Gfin,'Centroid','Area');
if(length(RegionG) > 0)
for s = 1:length(RegionG)
gx = RegionG(s).Centroid(1);%Find center of blob we made using pixel. Blob isnt perfect representation of Buoy
gy = RegionG(s).Centroid(2);
if (length(RegionY)>0 && ((((yy-gy)^2+(yx-gx)^2)^.5 <70 || RegionG(s).Area > 5300)))%If we have info for y, and we see green data near yellow, or huuggeee green data
    length(RegionY);
    ((yy-gy)^2+(yx-gx)^2)^.5;
    gy = yy;
    gx = yx;
    
    break
end
end    
    
    
%gx = RegionG(1).Centroid(1);
%gy = RegionG(1).Centroid(2);
end

%We know where like one pixel from the buoy is from rx and ry above, but we
%dont know the exact borders of it. RRR is the orinial probability map and
%has the whole buoy but also other noise. Take the rx and ry values to only
%look at the RRR image around there, blacken everything else

%But first create Ronly, an image that will have the whole buoy in white,
%plus some other noise
Ronly = RRR;
for i = 1:L
    for j = 1:W
        if (RRR(i,j)>.01)
            Ronly(i,j) = 255;
        end
    end
end

Ronly  = im2bw(Ronly);
rbound = 70;
if (length(RegionR) >0)
for aa = 1:L
    for bb = 1:W
        if (aa < ry-rbound|| aa > ry+rbound || bb < rx-rbound || bb > rx+rbound)%If anywhere in image besides close to the pixel from buoy
            Ronly(aa,bb) = 0;%Make black
        end
    end
end
end

Ronly = imfill(Ronly,'holes');%Fill holes
imshow(Ronly)
%Same for Yellow
Yonly = YYY;
for i = 1:L
    for j = 1:W
        if (YYY(i,j)>.01)
            Yonly(i,j) = 255;
        end
    end
end

%Creating image to be used for drawing boundaries on, includes nice buoy
%outline but also noise
if (length(RegionY) > 0)
for aa = 1:L
    for bb = 1:W
        if (aa < yy-40 || aa > yy+40 || bb < yx-40 || bb > yx+40)%If anywhere in image besides close to the pixel from buoy
            Yonly(aa,bb) = 0;%Make black
        end
    end
end
end
Yonly = im2bw(Yonly);%make binary
Yonly = imfill(Yonly,'holes');%Fill holes

imshow(Yonly)
%Yonly = imdilate(Yonly,se2);%round it out a little

Gonly = GGG;
%Make Gonly an image where the buoy is like pretty well defined
for i = 1:L
    for j = 1:W
        if (GGG(i,j)>1)
            Gonly(i,j) = 255;
        else
            Gonly(i,j) = 0;
        end
    end
end
if (length(RegionG) > 0)
for aa = 1:L
    for bb = 1:W
        if (aa < gy-15 || aa > gy+15 || bb < gx-15 || bb > gx+15)%If anywhere in image besides close to the pixel from buoy
            Gonly(aa,bb) = 0;%Make black
        end
    end
end
end
Gonly = im2bw(Gonly);%make binary
Gonly = imfill(Gonly,'holes');
Gonly = bwareaopen(Gonly,10);
Gonly = imdilate(Gonly,se3);

imshow(Gonly)
%Gonly = imdilate(Gonly,se2);%round it out a little

%Know green is in line (y axis) with other buoys
%If we see a blob for green buoy too high, black it out
for i = 1:L
    for j = 1:W
        if(i < ry-40)
            Gfin(i,j) = 0;
        end
    end
end




imshow(frame)
hold on

%Draw Countours
B = bwboundaries(Ronly);%Boundaries of the white in binary image
for  k =1:length(B)
boundary = B{k};
if (((yy-ry)^2+(yx-rx)^2)^.5 > 70 && abs(yx-rx)>70)%it's far away from yellow buoy
plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 3)%Plot the boundaries in red
end 
end

B = bwboundaries(Yonly);%Boundaries of the white in binary image
for  k =1:length(B)
boundary = B{k};
plot(boundary(:,2), boundary(:,1), 'y', 'LineWidth', 3)%Plot the boundaries in red
end 

B = bwboundaries(Gfin);%Boundaries of the white in binary image
Gcir = regionprops(Gfin,'Centroid','Area');
if (length(Gcir) > 0)
Ggreat = 0;
for i = 1:length(Gcir)
    if(abs(Gcir(i).Centroid(1) - yx) > 70&&abs(Gcir(i).Centroid(2)-yy < 20)...
            && abs(Gcir(i).Centroid(1)-rx > 40) )
        if (Gcir(i).Area > Ggreat)
        Ggreat = Gcir(i).Area;
        Gi = i;
        Gcx = Gcir(i).Centroid(1);
        Gcy = Gcir(i).Centroid(2);
        end
        scatter(Gcir(Gi).Centroid(1),Gcir(Gi).Centroid(2),400,'g','LineWidth',3)
    end
end



end

% 
% for  k =1:length(B)
% if (((yy-gy)^2+(yx-gx)^2)^.5 > 70 &&abs(gx-rx)>70)
% boundary = B{k};
% plot(boundary(:,2), boundary(:,1), 'g', 'LineWidth', 3)%Plot the boundaries in green
% end
% end 



%Transform matrix to vector and back
%AA = [ 1 2 3;
%       4 5 6;
%       7 8 9;]
%BB = reshape(AA,1,9) %makes a list, only 1 column
%CC = reshape(BB,3,3) %makes original AA matrix
title(n)
f = getframe(gca);
im = frame2im(f);
writeVideo(NewVid,im) %Add frame to video
set(gcf,'visible','off')
cla%clear axes, this makes it so it doesnt slow down over time by stacking images

end
close(NewVid);