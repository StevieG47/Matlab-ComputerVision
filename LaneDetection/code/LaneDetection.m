clc
clear all
% #Camera Matrix
% K = [[  1.15422732e+03   0.00000000e+00   6.71627794e+02]
% 
%  [  0.00000000e+00   1.14818221e+03   3.86046312e+02]
% 
%  [  0.00000000e+00   0.00000000e+00   1.00000000e+00]]
% 
% 
% 
% #Distortion Coefficients
% dist = [[ -2.42565104e-01  -4.77893070e-02
%   -1.31388084e-03  -8.79107779e-05
%     2.20573263e-02]]


vid = VideoReader('../input/carVid.mp4');%Read video into var vid
NewVid = VideoWriter('OUT');%Writin the new video
NewVid.FrameRate = 25;
open(NewVid);%open the new video
firstRun = 0;
fucked = 0;
prev = 0;%for previous values recording
Direction = 'LEFT';

for n = 370:370%Use 20 for a good curve, imshow(New)
frame = read(vid,n);%Read a specific frame of the video as an image
redChannel = frame(:, :, 1);
%imshow(frame)

%% GAUSSIAN FILTER
G = fspecial('gaussian', [5,5],2);
I = imfilter(frame,G,'same');%gaussian filter
I = rgb2gray(frame);%change to gray scale
%imshow(I);

%% EDGE DETECTION
edgeD = edge(I,'Roberts');%edge detection
[rows columns] = size(edgeD);%get dimensions of image
imshow(edgeD)

%% GET RID OF TOP HALF OF IMAGE
imArea = edgeD;%new var to start specifying area of interest
for i = 1:rows
    for j = 1:columns
        if (i<426)%if top half of image, no road there
            imArea(i,j) = 0;% get rid of any white pixels, dont care about that part of image
        end
    end
end
%imshow(imArea)

%Clean up image, get rid of smaller pixel areas
imAreaClean = bwareaopen(imArea,50);
imshow(imAreaClean)

         
%Get rid of pixels that arent high on the red scale, since lines are white
%and yellow
colorThresh = imAreaClean;
indexR = (redChannel < 100);
colorThresh(indexR) = 0;
imshow(colorThresh)

%Region props it
hold on
bad = [];
check = regionprops(colorThresh,'BoundingBox','Orientation');
for k = 1 : length(check)
  BB = check(k).BoundingBox;
  if(check(k).Orientation < 30 && check(k).Orientation > -20)%Check if they are horizonal ish
  rectangle('Position', BB,...
  'EdgeColor','r','LineWidth',2 )
   bad = [bad ; k];%Keep track of which regions are bad regions, too horizontal
 
  end    
end


%Make all pixels black in the bad regions
noHor = colorThresh;
for a = 1:length(bad)
    BB = int16(check(bad(a)).BoundingBox);
    for c = BB(1):BB(1)+BB(3)
        for r = BB(2):BB(2)+BB(4)
          noHor(r,c) = 0;
        end
    end
end
imshow(noHor)

%ISOLATE noHOR IMAGE TWO ONLY KEEP LANES, NO CARS OR NOTHING
for a = 1:columns
    for b = 1:rows
        if (b < 510 && a > 856 || a < 250)
            noHor(b,a) = 0;
        end
    end
end
            

%CUT OFF NOHOR IMAGE AT A Y LEVEL WE FIND TO BE APROPRIATE
for a = 1:rows
    for b = 1:columns
        if (b <450) 
            noHor(b,a) = 0;
        end
    end
end


%Get rid of small pixel groups, nosise and dilate, then fill
se1 = strel('disk',3);
New = bwareaopen(noHor,20);
New = imdilate(New,se1);
New = imfill(New,'holes');
imshow(New)
    hold on

 %Get regions of image, should mostly be lanes
check2 = regionprops(New,'BoundingBox','Orientation','Eccentricity');
for k = 1 : length(check2)
  BB = check2(k).BoundingBox;
  %if(check2(k).Orientation < 30 && check(k).Orientation > -20)%Check if they are horizonal ish
  %rectangle('Position', BB,...
  %'EdgeColor','r','LineWidth',2 )
  check2(k).Orientation;
  Ecc = check2(k).Eccentricity;
  BB;
    %end    
end





%% HOUGH
%Make new totally black image
houghIm = noHor;
index = (redChannel > -1);
houghIm(index) = 0;

%Hough Transform
[H,T,R] = hough(noHor); % get transformatio H, theta T, rho R

%Hough Peaks
P  = houghpeaks(H ,10,'threshold',ceil(0.1*max(H(:))));
%P  = houghpeaks(H,10,'threshold', 0);
x = T(P(:,2)); y = R(P(:,1));
plot(x,y,'s','color','white');

%Hough lines, plot them
lines = houghlines(noHor,T,R,P,'MinLength',15);
%figure
imshow(noHor), hold on %what figure to plot lines over
max_len = 0;
lineTrack = [];
posSlope = [];
negSlope = [];
Pslope = [];
Nslope = [];
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','red');
   if( abs(xy(1,2)-xy(2,2)) > 2)%no stray horizontals, check endpoint y difference
   lineTrack = [lineTrack; xy];%Keep track of xy endpoints of each line, each line gives 2x2
   
  
   
     %Lines always listed in increasing x, so check y values to find if
   %positive or negative slope, add to group
     if (xy(1,2)>xy(2,2) && xy(1,1)<xy(2,1))%pos slope
       posSlope = [posSlope ; xy];
   end
   
   if (xy(1,2)<xy(2,2) && xy(1,1)<xy(2,1))%neg slope
       negSlope = [negSlope ; xy];
   end
   end
 
%    % Plot beginnings and ends of lines
%    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
end

 for i = 1:2:length(lineTrack)
       if (lineTrack(i,1) < 630)%if left lane line
           Pslope = [Pslope ; [lineTrack(i,1) lineTrack(i,2)] ; [lineTrack(i+1,1) lineTrack(i+1,2)]];
        end
           
        
       if (lineTrack(i,1) > 630)
           Nslope = [Nslope ; [lineTrack(i,1) lineTrack(i,2)]; [lineTrack(i+1,1) lineTrack(i+1,2)]];   
       end
 end

%Using Pslope and N slope, find min and max x,y values to plot line (which
%will be extrapolated)
minX = min(Pslope(:,1));
maxX= max(Pslope(:,1));
minY = min(Pslope(:,2));
maxY= max(Pslope(:,2));

minXN = min(Nslope(:,1));
maxXN= max(Nslope(:,1));
minYN = min(Nslope(:,2));
maxYN= max(Nslope(:,2));


%Look at previous bottom x values for each lane. If lane has jumped a bunch
%we then move it in the direction in jumped but not by that much. Like
%some kinda of filter
% if (prev ==1)
%     if (abs(minX-minXprev) > 3 && minX < minXprev)
%         minX = minXprev - 1;
%         minY = minYprev;
%         maxX = maxXprev + 1;
%         maxY = maxYprev;
%     end
%     if (abs(minX-minXprev) > 1 && minX > minXprev)
%         minX = minXprev + 1;
%         minY = minYprev;
%         maxX = maxXprev -1;
%         maxY = maxYprev;
%     end
% %     if (abs(maxXN-maxXNprev) > 5 && maxXN < maxXNprev)
% %         maxXN = maxXNprev - 1
% %     end
% %      if (abs(maxXN-maxXNprev) > 5 && maxXN > maxXNprev)
% %         maxXN = maxXNprev + 1
% %     end
% %     maxXNprev
%end

%REcord previous values for next run
if (firstRun == 1)   
    minXprev = minX;
    maxXprev = maxX;
    minYprev = minY;
    maxYprev = maxY;
    
%     minXNprev = minXN;
%     maxXNprev = maxXN;
%     minYNprev = minYN;
%     maxYNprev = maxYN;
    prev = 1;
end


% PLOTTING
%Set new image since we plotted on top of an image
%This  will be extrapolated/ interpolated lines from the ones in Lanes
frameGrab = getframe(gca);
Lanes = frame2im(frameGrab);
%figure
LaneExtrap = Lanes;
%Want an all black image to start with. We are only extrapolating the
%largest line for each slope, so we dont want the other smaller lines in
%the picture
for q = 1:columns
    for w = 1:rows
              LaneExtrap(w,q) = 0;%Make all black image
     end
end
imshow(LaneExtrap)
hold on

%Find the largest line in pos slope and neg slope, only want to extrapolate
%one line for each
%Pos
maxLine = 0;
for p = 1:2:size(posSlope)%each line is 2x2, endpoints (x1,y1);(x2,y2) So cycle through by two each time
x1 = posSlope(p,1);
x2 = posSlope(p+1,1);
y1 = posSlope(p,2);
y2 = posSlope(p+1,2);
lineDist = ((y2-y1)^2 +(x2-x1)^2)^.5;%get the length of the line

if (lineDist > maxLine)%if the length of the line is the new max, save endpoints
    end_pointsX = [posSlope(p,1) posSlope(p+1,1)];
    end_pointsY = [posSlope(p,2) posSlope(p+1,2)];
end
end

%Neg
maxLine = 0;
for p = 1:2:size(negSlope)%each line is 2x2, endpoints (x1,y1);(x2,y2) So cycle through by two each time
x1 = negSlope(p,1);
x2 = negSlope(p+1,1);
y1 = negSlope(p,2);
y2 = negSlope(p+1,2);
lineDist = ((y2-y1)^2 +(x2-x1)^2)^.5;%get the length of the line

if (lineDist > maxLine)%if the length of the line is the new max, save endpoints
    end_pointsXneg = [negSlope(p,1) negSlope(p+1,1)];
    end_pointsYneg = [negSlope(p,2) negSlope(p+1,2)];
end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%OLD WAY. THIS EXTRAPOLATES EVERY LINE AND PLOTS EVERY LINE NOT JUST THE
%BIGGEST ONE
%set X as x values of endpoints of line,Y as y values of endpoints of line
%YI are y values for every x value, interpolated line
%Image will have red line for every x value
% for  z = 1:2:size(lineTrack)
%     X = [lineTrack(z,1) lineTrack(z+1,1)];
%     Y = [lineTrack(z,2) lineTrack(z+1,2)];
%     XI = [0:1:columns];
%     YI = interp1(X,Y,XI,'linear','extrap');%interpolate lines
%     plot(XI,YI,'LineWidth',5,'Color','red')%Plot the lines
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Plot the positive longest line extrapolated/interpolated
X = end_pointsX;
Y = end_pointsY;
%NEW X AND Y METHOD WITH Pslope INSTEAD OF posSlope
X = [minX maxX];
Y = [maxY minY];
XI = [0:1:columns];
YI = interp1(X,Y,XI,'linear','extrap');%interpolate lines
plot(XI,YI,'LineWidth',5,'Color','red')%Plot the lines

%Plot the negative slope longest line extapolated/interpolated
Xneg = end_pointsXneg;
Yneg = end_pointsYneg;
%NEW X AND Y METHOD WITH Nslope INSTEAD OF negSlope
Xneg = [minXN+5 maxXN];
Yneg = [minYN maxYN];
XIneg = [0:1:columns];
YIneg = interp1(Xneg,Yneg,XIneg,'linear','extrap');%interpolate lines
plot(XIneg,YIneg,'LineWidth',5,'Color','red')%Plot the lines

%Find the minimum y value, want to cut off where we plot so
%lanes lines dont criss cross
%If something fucked up assume the last frames minY
minY = min(lineTrack(:,2));
minY = 460;
maxY = 670;% max(lineTrack(:,2));s
if (firstRun == 1)
   if (abs(minY-minYprev) > 20)
       minY = minYprev;
       fucked = fucked+1
   end
end
minYprev = minY;




plottedY = [end_pointsY end_pointsYneg];
%minY =  min(plottedY);
%maxY = max(plottedY);

%Cut off everything below minY, towards the top of the image
%Cut off everything above maxY towards bottom of image
frameGrab = getframe(gca);
finalLanes = frame2im(frameGrab);

for q = 1:columns
    for w = 1:rows
        if (w < minY  || w > maxY)
            finalLanes(w,q) = 0;
        end
    end
end
%figure 
size(finalLanes)
imshow(finalLanes);

framewLines = finalLanes;%Actual frame of video with lines on them
%Put red lines on original frame now
for a = 1:columns
    for b = 1:rows
        if (finalLanes(b,a,1) <5)%check red channel since we plotted lines in red
            framewLines(b,a,1) = frame(b,a,1);%if not red, change pixel to frame pixel
            framewLines(b,a,2) = frame(b,a,2);
            framewLines(b,a,3) = frame(b,a,3);
        end
    end
end

%%FILL LANE BETWEEN RED LINES WITH COLOR
%redLeft1 is left side of left red line,  2 is right side of left 
%Cycle through pixels across when we first hit perfect red mark it with
%redLeft1, when we hit a non perfect red we know the line is over, mark it
%with red Left2 same with right. Color when redLeft 2 triggers and before
%redRight1 triggers
laneFill = framewLines;
for y = minY:maxY
    redLeft1 = 0;
    redLeft2 = 0;
    redRight1 = 0;
    redRight2 = 0;
    for x = 1:1200
        if (redLeft1 == 0 && redLeft2 == 0 && framewLines(y,x,1) == 255 && framewLines(y,x,3) < 1)
            redLeft1 = 1;
        end
        if (redLeft1 == 1 && redLeft2 == 0 && framewLines(y,x,2) >1 && framewLines(y,x+1,2) > 1 )
            redLeft2 = 1;
        end
        if(redLeft2 == 1 && redRight1 == 0 && framewLines(y,x,1) == 255 && framewLines(y,x,3) < 1)
            redRight1 = 1;
        end
        if (redLeft2 ==1 && redRight1 == 0)
           laneFill(y,x,1) = 0;
           laneFill(y,x,2) = 0;
           laneFill(y,x,3) = 255;
        end
        
    end
end


%PUT TRANSPARENCY COLOR ON LANE FROM STACKOVERFLOW
finalLane = frame;
for x = 1:1200
    %pt1 is bottom left of lane
    if(laneFill(maxY,x,3) == 255 && laneFill(maxY,x,1) == 0 && laneFill(maxY,x-1,3) == 0)
        pt1 = [x, maxY];
    end
    
    %pt2 is top left of lane
    if(laneFill(minY,x,3) == 255 && laneFill(minY,x,1) == 0 && laneFill(minY,x-1,3) == 0)
        pt2 = [x, minY];
    end
    
    %pt3 is top right of lane
    if(laneFill(minY,x,3) == 255 && laneFill(minY,x,1) == 0 && laneFill(minY,x+1,1) == 255)
        pt3 = [x, minY];
    end
    
    %pt4 is bottom right of lane
    if(laneFill(maxY,x,3) == 255 && laneFill(maxY,x,1) == 0 && laneFill(maxY,x+1,1) == 255)
        pt4 = [x, maxY];
    end
  
end


%     pt1 = [342, 670];
%     pt2 = [605, 450];
%     pt3 = [676, 451];
%     pt4 = [1100, 667];
    pt_x = [pt1(1) pt2(1) pt3(1) pt4(1)];
    pt_y = [pt1(2) pt2(2) pt3(2) pt4(2)];
    BW = poly2mask(pt_x, pt_y, 720, 1280);
    
    
    clr = [255 0 255];            % color of lane
    a = 0.3;                 % play around with 
    z = false(size(BW));
    
    
    mask = cat(3,BW,z,z); finalLane(mask) = a*clr(1) + (1-a)*finalLane(mask);
    mask = cat(3,z,BW,z); finalLane(mask) = a*clr(2) + (1-a)*finalLane(mask);
    mask = cat(3,z,z,BW); finalLane(mask) = a*clr(3) + (1-a)*finalLane(mask);
    
  
 
  %Take red lines from framewLines and put them on finaLanes pic with
  %transparent stuff
  for a = 1:columns
      for b = 1:rows
          if (framewLines(b,a,1) == 255 && framewLines(b,a,2) == 0 && framewLines(b,a,3) == 0)%if red line
              finalLane(b,a,1) = 255;
              finalLane(b,a,2) = 0;
              finalLane(b,a,3) = 0;
          end
      end
  end
 
imshow(finalLane)
%imshow(noHor)

%FINDING INTERSECTION OF HOUGH LINES

x1 = [pt_x(1) pt_x(2)];
y1 = [pt_y(1) pt_y(2)];
x2 = [pt_x(3) pt_x(4)];
y2 = [pt_y(3) pt_y(4)];

%fit linear polynomial
p1 = polyfit(x1,y1,1);
p2 = polyfit(x2,y2,1);

%calculate intersection
x_intersect = fzero(@(x) polyval(p1-p2,x),3);
y_intersect = polyval(p1,x_intersect);

%plot intersection
% imshow(noHor)
% hold on
% line(x1,y1);
% hold on;
% line(x2,y2);
% plot(x_intersect,y_intersect,'r*')

%USE INTERSECTION TO DECIDE IF TURN OR NA
%LEft of x = 616? for left
%Stragiht was like x = 642
if (x_intersect < 630)
    Direction = 'Left'
end
if (x_intersect > 630)
    Direction = 'Straight'
end
if (x_intersect > 680)
    Direction = 'Right'
end
tex = text(600,150,Direction,'Color','k','FontSize',30); 
frameGrab = getframe(gca);
videoWrite = frame2im(frameGrab);
%figure
firstRun = 1;
%imshow(framewLines)
%imshow(laneFill)
writeVideo(NewVid,videoWrite) %Add frame to video
%cla
n
end
close(NewVid);
fucked


            
            


% X = [851 972];
% Y = [544 613];
% XI = [851 1200];
% YI = interp1(X,Y,XI,'linear','extrap');
% figure
% imshow(Lanes)
% hold on
% plot(XI,YI,'LineWidth',5,'Color','red')



