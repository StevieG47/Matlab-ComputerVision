clc
close all
%clear all


warning('off','images:initSize:adjustingMag')

%CREATE VIDEO 
A = VideoWriter('asdfasdfasdf');
open(A)
RedClass = VideoWriter('Dareprot.mp4');
RedClass.FrameRate = 30;
open(RedClass);
test = 0; %for clearing windows, 1 to clear them, 0 to keep
foundRed = 0;
bikeprev = 0;
bikeprevx = 0;
bikeprevy = 0;
pprev = 0;
pprevx = 0;
pprevy = 0;
Counte = 0;
colPrev = 10;

vid = VideoReader('Input.avi');
startframe = 2125;
endframe =   2125;
for n = startframe:1:endframe %2861 total, 
    frame = read(vid,n);
    frameO=frame;
    [rows columns,re] = size(frame);
    redChannel = frame(:,:,1); 
    greenChannel = frame(:,:,2);
    blueChannel = frame(:,:,3);
   
    %imshow(frame)
    
%% GAUSSIAN FILTER
G = fspecial('gaussian', [5,5],2);
I = imfilter(frame,G,'same');%gaussian filter
%imshow(I)
%% STRETCHLIM IMAGE
J = imadjust(I,stretchlim(I),[]);
%figure
%imshow(J)
%figure
%% CREATE APPROPRIATE GRAYSCALE
gray = 3*redChannel-.5*blueChannel-2*greenChannel;
%gray = 2.*redChannel-blueChannel-greenChannel;
%gray = redChannel;
%imshow(gray)
%% NORMALIZE INTENSITY OF IMAGE
mini = 0;
maxNormal = 255;
normalized = gray*(maxNormal/max(gray(:)));
%figure
%imshow(normalized)
%% THRESHOLD NORMALIZED IMAGE
Norm = normalized;   
%Norm(index) = 0;
%Norm = im2bw(Norm);
%se1 = strel('disk',1);
%Norm = imdilate(Norm,se1);
%Norm = bwareaopen(Norm,4);
%index2 = (Norm == 0);
%Norm2 = normalized;
%Norm2(index2) = 0;
%imshow(Norm)
%% MSER DETECTION RED
[mserRegions, mserConnComp] = detectMSERFeatures(Norm, ...
    'RegionAreaRange',[250 6000],'ThresholdDelta',9);


figure
imshow(Norm)

hold on
plot(mserRegions, 'showPixelList', true,'showEllipses',false)
title('MSER regions')
%figure
%imshow(frame)
%hold on

%% USE REGIONPROPS TO MEASURE MSER PROPERTIES RED
mserStats = regionprops(mserConnComp, 'BoundingBox', 'Eccentricity', ...
    'Solidity', 'Extent', 'EulerNumber', 'Image','Area');



%FILTER OUT BASED ON PROPERTIES OF REGION
%Aspect Ratio
bbox = vertcat(mserStats.BoundingBox);
if length(bbox >0)
w = bbox(:,3);
h = bbox(:,4);
aspectRatio = w./h;
end

filterIdx = [mserStats.Eccentricity] > .685 ; %Eccentricity of 0 is circle, 1 is line. Signs will never be lines so want it above lets say .6
filterIdx = filterIdx | [mserStats.Extent] < 0.2 | [mserStats.Extent] > 0.9;%Region area/Area of bounding box. Really small means big box over small region
%filterIdx = filterIdx | [mserStats.EulerNumber] < -4;
if (length(bbox) > 0)
filterIdx = filterIdx | aspectRatio' > 1.5;
filterIdx = filterIdx | aspectRatio' < .65;
end

%REMOVE REGIONS
mserStats(filterIdx) = [];
Size = size(mserStats);
if (Size(2) > 0)%Can only go on if there are still entries
mserRegions(filterIdx) = [];

%If below certain y value (above certain y value) we know sign must be
%kinda big/small. Trash if not
remove = [];
for i = 1:length(mserStats)
    if (mserStats(i).BoundingBox(2) < 35)
    if mserStats(i).Area > 650
        % remove = [remove; i];
         0 ;        
    end
    end
    if (mserStats(i).BoundingBox(2) < 60)
    if mserStats(i).Area > 710
         remove = [remove; i];
         .5   ;    
    end
    end
    if (mserStats(i).BoundingBox(2) < 80)
    if mserStats(i).Area > 1200
         remove = [remove; i];
         1;
         
    end
    end
     if (mserStats(i).BoundingBox(2) < 100)
    if mserStats(i).Area > 3500
         remove = [remove; i];
         2;
         
    end
    end
   if (mserStats(i).BoundingBox(2) < 180)
    if mserStats(i).Area > 5700
         remove = [remove; i];
         3;
         
    end
   end
    if (mserStats(i).BoundingBox(2) < 180)
    if mserStats(i).Area < 310
         remove = [remove; i];
         4;
    end
    end
   if (mserStats(i).BoundingBox(2) > 150)
    if mserStats(i).Area < 2200
         remove = [remove; i];
         5;
    end
   end
   if (mserStats(i).BoundingBox(2) > 180)
    if mserStats(i).Area < 500
         remove = [remove; i];
         6;
    end
   end
   if (mserStats(i).BoundingBox(2) > 280)
    if mserStats(i).Area < 700
         remove = [remove; i];
         7;
    end
   end
   if (mserStats(i).BoundingBox(2) > 310)
    if mserStats(i).Area < 2500
         remove = [remove; i];
         7.5;
    end
   end
if (mserStats(i).BoundingBox(2) > 385)
    if mserStats(i).Area < 1500
        remove = [remove; i];
        8;
    end
end
if (mserStats(i).BoundingBox(2) > 420)
    if mserStats(i).Area < 2250-0.1405-0.1405-0.1405
        remove = [remove; i];
        9;
    end
end
if (mserStats(i).BoundingBox(2) > 505)
    if mserStats(i).Area < 3800
        remove = [remove; i];
        10;
    end
end
if (mserStats(i).BoundingBox(2) < 2|| mserStats(i).BoundingBox(1)+mserStats(i).BoundingBox(3) > 1625 || mserStats(i).BoundingBox(1) < 5)
         remove = [remove; i]; 
         11;
end

% %TESTINGGGG
% if (mserStats(i).BoundingBox(1) < 500)
%          remove = [remove; i]; 
% end
% %%%%%%%%%%%%%%%%%%%TESTING
end

%CROP BOX FOR FURTHRER ANALYSIS
for i = 1:length(mserStats)
cropR = imcrop(frame,mserStats(i).BoundingBox);
meanRed = mean2(cropR(:,:,1));
meanGreen = mean2(cropR(:,:,2));
meanBlue = mean2(cropR(:,:,3));
    if ( (meanRed < meanBlue || meanGreen > meanRed) && meanRed <140) %meanRed < 105 ||
        mserStats(i).BoundingBox;
        meanRed;
        remove = [remove ; i];
        'remove r < g/b';
    end
    if (meanGreen < 70 || meanBlue < 82 || meanRed < 87 || abs(meanRed-meanGreen)<1)%Signs have white in them, false positives dont (many of them)
        remove = [remove; i];
        'remove g/b';
    end
    b = cropR(:,:,3);
    c = numel(find(b > 250));%signs have white in them, 255,255,255
    if (c < 3 && mserStats(i).Area >685)%MAYBE MAKE THIS FOR INTONLT
        remove = [remove;i];
        c;
    end
end

mserStats(remove) = [];

%REMOVE BOXES WITHIN EACH OTHER
%Compare every box with each other. If any are within another remove the
%one within
track = [];
for a = 1:length(mserStats)
    for b = 1:length(mserStats)
        BB1 = mserStats(b).BoundingBox;
        BB2 = mserStats(a).BoundingBox;
        if (BB2(1) < BB1(1) && BB1(1) < BB2(1)+BB2(3)&& BB2(2) < BB1(2) && BB1(2) < BB2(2)+BB2(4))% if x wihtin x and y within y
            track = [track; b];
        end
        if (BB1(2) == BB2(2) && BB1(3) > BB2(3))
            track = [track;b];
        end
    end
end
mserStats(track) = [];%Delete regions that were within others
% % imshow(frame)
% % hold on
% % for k = 1 : length(mserStats)
% %             BB = mserStats(k).BoundingBox;
% %             rectangle('Position', BB,'EdgeColor','r','LineWidth',2);
% % end



%SHOW REMAINING REGIONS
%figure
imshow(frame)      
%size(frame);
%hold on
mserStatsRed = mserStats;
foundRed =1; %We know mserStats for red isnt empty
% for k = 1 : length(mserStats)
%   BB = mserStats(k).BoundingBox;
%   rectangle('Position', BB,'EdgeColor','r','LineWidth',2);
% end
end

%% CLASSIFY RED
imshow(frame)
hold on
plotBoxR = [];
textTrackR = [];
messageTrackR = [];
score1 = -99;
for q = 1:length(mserStats)
    BBred = mserStats(q).BoundingBox;
    testingFeatures = zeros(1, hogFeatureSize, 'single');
    imgR = imcrop(frame,BBred);
    cropR = imresize(imgR,[64 64]);
    Gred = rgb2gray(cropR);
    Binred = imbinarize(Gred);
    testingFeatures(1,:) = extractHOGFeatures(Binred,'CellSize', cellSize);
    Sizer = size(testingFeatures);
    testingLabels = zeros(Sizer(1),1);
    
% Make class predictions using the test features.
    predictedLabels = predict(classifier, testingFeatures);
    [p,score] = predict(classifier, testingFeatures);
    B = score;
    [maxi,col] = max(B);

% Tabulate the results using a confusion matrix.
    confMat = confusionmat(testingLabels, predictedLabels);
    confMat = bsxfun(@rdivide,confMat,sum(confMat,2));%Make it percentages out of 1

%GET WHICH IMAGE IT IS
confMatT = confMat';
[M,I] = max(confMatT); 
%figure
%if (I(1) == 1)%WE GOTTA STOP SIGN
%colPrev
if (colPrev == 1 && maxi>-.12 && score(2) < -.25)
    col = 1;
    col;
end
if (colPrev == 4 && maxi > -.29)
    col = 4;
end
if (colPrev == 2 && maxi > -.29)
    col = 2;
end


if (col == 1)
    'Intersection'
   %if (max(score) > -.23)
   %if (score(1) > score1)
    %if (((stopPrev(1)-BBred(1))^2 + (stopPrev(2)-BBred(2))^2)^.5 < 10)
    if (B(4) >-.15)
        B(2) = B(1);
        col = 4;
    end
    if (BBred(1) < 430 && BBred(2) < 120)
            break
    end
    if (BBred(1) < 120 && BBred(2) < 180)
            break
    end
    if (maxi > -.115 && abs(B(1)-B(2))>.001 && abs(B(1)-B(3))>.001 && BBred(2)>35)
        picInsert = imresize(ident1,[BBred(4) BBred(3)]);
        red = cropR(:,:,1);
        meanr = mean2(red);
        %if (meanr > 96)
        Psize = size(picInsert);
        startRow = int16(BBred(2));
        if (startRow < 0)
            startRow = 5;
        end
        startColumn = int16(BBred(1)- BBred(3)); %BBred(3)-1);
        if (startColumn > 0 && startColumn < columns)
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,1) = picInsert(:,:,1);%INSERT PIC IN RIGHT POSITION
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,2) = picInsert(:,:,2);
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,3) = picInsert(:,:,3);
        imshow(frame)
        tx = double(startColumn)-Psize(2)-20 ; ty = double(startRow+Psize(1))+20;
        message = 'S';
        tex = text(tx,ty,'INTERSECTION','Color','r','FontSize',10,'FontWeight','bold');
         plotBoxR = [plotBoxR; BBred];
      textTrackR = [textTrackR ; tx ty];
      messageTrackR = [messageTrackR; message];
      colR = col;
        end
      %stopPrev = BBred;
        %end
       % end
        end
      %score1 = score(1);
  % end
       % end
end
%if(I(1) == 2) %WE GOTTA UH INTERSECTION THING SIGN
if(col ==2)
    'Bump';
    if (maxi > -.18)
    picInsert = imresize(bumpIm,[100 100]);
        red = cropR(:,:,1);
        meanr = mean2(red);
        Psize = size(picInsert);
        startRow = int16(BBred(2)-20);
        if (startRow < 0)
            startRow = 5;
        end
        startColumn = int16(BBred(1)-110-1);
        if (startColumn > 0 && startColumn < columns)
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,1) = picInsert(:,:,1);%INSERT PIC IN RIGHT POSITION
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,2) = picInsert(:,:,2);
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,3) = picInsert(:,:,3);
        imshow(frame)
        tx = double(startColumn)-Psize(2)-20 ; ty = double(startRow+Psize(1))+20;
        message = 'S';
        tex = text(tx,ty,'SPEED BUMP','Color','r','FontSize',10,'FontWeight','bold');
         plotBoxR = [plotBoxR; BBred];
      textTrackR = [textTrackR ; tx ty];
      messageTrackR = [messageTrackR; message];
      colR = col;
       % end
        end
       
    end
end
if(col ==3)
    'YIELD';
    if (maxi > -.099)
        if (BBred(1) < 385 && BBred(2) < 70)
            break
        end
    picInsert = imresize(yieldIm,[100 100]);
        red = cropR(:,:,1);
        meanr = mean2(red);
        Psize = size(picInsert);
        startRow = int16(BBred(2)-20);
        startColumn = int16(BBred(1)-110-1);
        if (startColumn > 0 && startColumn < columns)
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,1) = picInsert(:,:,1);%INSERT PIC IN RIGHT POSITION
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,2) = picInsert(:,:,2);
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,3) = picInsert(:,:,3);
        imshow(frame)
        tx = double(startColumn)-Psize(2)-20 ; ty = double(startRow+Psize(1))+20;
        message = 'S';
        tex = text(tx,ty,'YIELD','Color','r','FontSize',10,'FontWeight','bold');
         plotBoxR = [plotBoxR; BBred];
      textTrackR = [textTrackR ; tx ty];
      messageTrackR = [messageTrackR; message];
      colR = col;
       % end
        end
       
    end
end
col
if(col ==4)
    'LANE MERGE';
    if (maxi > -.3)
    picInsert = imresize(mergeIm,[100 100]);
        red = cropR(:,:,1);
        meanr = mean2(red);
        Psize = size(picInsert);
        startRow = int16(BBred(2)-20);
        startColumn = int16(BBred(1)-110-1);
        if (startColumn > 0 && startColumn < columns)
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,1) = picInsert(:,:,1);%INSERT PIC IN RIGHT POSITION
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,2) = picInsert(:,:,2);
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,3) = picInsert(:,:,3);
        imshow(frame)
        tx = double(startColumn)-Psize(2)-20 ; ty = double(startRow+Psize(1))+20;
        message = 'S';
        tex = text(tx,ty,'LANE MERGE','Color','r','FontSize',10,'FontWeight','bold');
         plotBoxR = [plotBoxR; BBred];
      textTrackR = [textTrackR ; tx ty];
      messageTrackR = [messageTrackR; message];
      colR = col;
       % end
        end
       
    end
end
 % imshow(ident2)
end

%USE TO VIEW SVM RESULTS
%DisplayConfusionMatrix(confMat); 



% frameCurr = getframe(gca);
% RedDone = frame2im(frameCurr);
% size(frame)
% size(RedDone)
%% BLUE TIME
%% CREATE APPROPRIATE GRAYSCALE
%gray = 2*blueChannel-greenChannel-redChannel;
gray = 3*blueChannel-greenChannel-2*redChannel;
%gray = 2.*redChannel-blueChannel-greenChannel;
%gray = redChannel;
%imshow(gray)
%% NORMALIZE INTENSITY OF IMAGE
min = 0;
maxNormal = 255;
normalized = gray*(maxNormal/max(gray(:)));
Norm = normalized;
index = (normalized > 80);
Norm(index) = 255;
Norm1 = im2bw(Norm);
Norm = bwareaopen(Norm1,12);
se1 = strel('disk',5);
Norm = imdilate(Norm,se1);
ind2 = (normalized < 120);
Norm2(ind2) = 0;
Norm2 = bwareaopen(Norm1,20);
%figure
%% MSER DETECTION BLUE
[mserRegions, mserConnComp] = detectMSERFeatures(normalized, ...
    'RegionAreaRange',[1 6000],'ThresholdDelta',5);


imshow(normalized)
hold on
plot(mserRegions, 'showPixelList', true,'showEllipses',false)
title('MSER regions')
figure

%% USE REGIONPROPS TO MEASURE MSER PROPERTIES BLUE
blueStats = regionprops(mserConnComp, 'BoundingBox', 'Eccentricity', ...
    'Solidity', 'Extent', 'EulerNumber', 'Image','Area');
% for c = 1:length(blueStats)
%     BB = blueStats(c).BoundingBox;
%     rectangle('Position', BB,'EdgeColor','y','LineWidth',2);
% end

Thresh = regionprops(Norm,'BoundingBox', 'Eccentricity', ...
    'Solidity', 'Extent', 'EulerNumber', 'Image','Area');
 %figure
% imshow(Norm)
% hold on
% for c = 1:length(Thresh)
%     BB = Thresh(c).BoundingBox;
%     rectangle('Position', BB,'EdgeColor','y','LineWidth',2);
% end
    
%COMBINE OVERLAPPING BOXES
% Boxes = [];
% for i = 1:length(blueStats)
% Boxes = [Boxes ; blueStats(i).BoundingBox];
% end
% BoxRemo = [];
% BoxAddo = [];
% for a = 1:length(blueStats)
%     for b = 1:length(blueStats)
%         Box1 = blueStats(a).BoundingBox;
%         Box2 = blueStats(b).BoundingBox;
%         %if right corner of 2 inside 1
%         if (Box1(1) < Box2(1)+Box2(3) && Box2(1)+Box2(3) < Box1(1)+Box1(3))
%             combineB = [Box2(1) Box1(2) Box1(1)+Box1(3)-Box2(1) Box2(4)+Box2(2)-Box1(2)];
%             %Boxes(a,:) = []; Boxes(b,:) = [];
%             %Boxes = [Boxes; combineB];
%             BoxRemo = [BoxRemo; a; b];
%             BoxAddo = [BoxAddo; combineB];
%         end
%     end
% end
% Boxes(BoxRemo,:) = [];
% Boxes = [Boxes ; combineB];

%FILTER OUT BASED ON PROPERTIES OF REGION
%Aspect Ratio
bbox = vertcat(blueStats.BoundingBox);
if length(bbox >0)
w = bbox(:,3);
h = bbox(:,4);
aspectRatio = w./h;
end

filterIdxB = [blueStats.Eccentricity] > .685 ; %Eccentricity of 0 is circle, 1 is line. Signs will never be lines so want it above lets say .6
filterIdxB = filterIdxB | [blueStats.Extent] < 0.2 | [blueStats.Extent] > 0.9;%Region area/Area of bounding box. Really small means big box over small region
%filterIdx = filterIdx | [mserStats.EulerNumber] < -4;
if (length(bbox) > 0)
filterIdxB= filterIdxB | aspectRatio' > 1.5;
filterIdxB = filterIdxB | aspectRatio' < .65;
end

%REMOVE REGIONS
%blueStats(filterIdxB) = [];
Size = size(blueStats);
%if (Size(2) > 0)%Can only go on if there are still entries
%mserRegions(filterIdxB) = [];


%figure
imshow(frame)
% hold on
% Boxsize = size(Boxes);
% for i = 1:Boxsize(1)
%     BB = Boxes(i,:);
%     rectangle('Position', BB,'EdgeColor','y','LineWidth',2);
% end



%THRESHOLDS FOR BLUE SIGNS
% Thresh = regionprops(Norm, 'BoundingBox', 'Eccentricity', ...
%     'Solidity', 'Extent', 'EulerNumber', 'Image','Area');
% %Aspect Ratio and region props filters
% 
% figure
% imshow(frame)
% hold on
% bbox = vertcat(Thresh.BoundingBox);
% if length(bbox >0)
% w = bbox(:,3);
% h = bbox(:,4);
% aRatio = w./h;
% filter = aRatio' > 1.5;
% filter = filter| aRatio' < .65;
% filter = filter | [Thresh.Area] < 250 ;
% filter = filter | [Thresh.Eccentricity] > .85 ;
% Thresh(filter) = [];
% end
% 
% 
% removeBlue = [];
% %AS Y VALUE CHANGES WE KNOW SIGN MUST BE CERTAIN AREAS, CANT BE HUGE IF AT
% %LIKE TOP OF IMAGE, CANT BE SMALL IF RIGHT IN FRONT OF US
% for i = 1:length(Thresh)
% %     if (Thresh(i).BoundingBox(2) < 50)
% %     if (Thresh(i).Area > 1000)
% %          removeBlue = [removeBlue; i];   
% %     end
% %     end
%      if (Thresh(i).BoundingBox(2) < 70)
%     if (Thresh(i).Area > 660)
%          removeBlue = [removeBlue; i];   
%     end
%      end
%     if (Thresh(i).BoundingBox(2) < 100)
%     if (Thresh(i).Area < 530)
%          removeBlue = [removeBlue; i];   
%     end
%     end
%     if (Thresh(i).BoundingBox(2) < 160)
%     if (Thresh(i).Area < 435)
%          removeBlue = [removeBlue; i];   
%     end
%     end
%     if (Thresh(i).BoundingBox(2) > 400)
%     if (Thresh(i).Area < 1200)
%          removeBlue = [removeBlue; i];   
%     end
%     end
%      if (Thresh(i).BoundingBox(2) > 500)
%     if (Thresh(i).Area < 2000)
%          removeBlue = [removeBlue; i];  
%     end
%      end
%     if (Thresh(i).BoundingBox(2) < 2|| Thresh(i).BoundingBox(1)+Thresh(i).BoundingBox(3) > 1624 || Thresh(i).BoundingBox(1) < 5)
%          removeBlue = [removeBlue; i]; 
%     end
% end
% %Thresh(removeBlue) = [];
% 
% %CROP BOX FOR FURTHRER ANALYSIS
% for i = 1:length(Thresh)
% crop = imcrop(frame,Thresh(i).BoundingBox);
% meanRed = mean2(crop(:,:,1));
% meanBlue = mean2(crop(:,:,3));
% meanGreen = mean2(crop(:,:,2));
%     if (meanGreen > meanBlue)
%         removeBlue = [removeBlue ; i];
%     end
% end
% Thresh(removeBlue) = [];
% 
% %REMOVE BOXES WITHIN EACH OTHER
% %Compare every box with each other. If any are within another remove the
% %one within
% track = [];
% for a = 1:length(Thresh)
%     for b = 1:length(Thresh)
%         BB1 = Thresh(b).BoundingBox;
%         BB2 = Thresh(a).BoundingBox;
%         if (BB2(1) < BB1(1) && BB1(1) < BB2(1)+BB2(3)&& BB2(2) < BB1(2) && BB1(2) < BB2(2)+BB2(4))% if x wihtin x and y within y
%             track = [track; b];
%         end
%     end
% end
% Thresh(track) = [];%Delete regions that were within others
% 
% %SHOW MERGED
% % Show the final text detection result.
% imshow(frame)
% hold on
 plotBox = [];
% textTrack = [];
% messageTrack = [];
%Boxisize = size(Boxes);
ThreshKeep = [];
% for a = 1:length(Thresh)%check thresh BB vs mser BB
%     for b = 1:length(blueStats)
%         Tx = Thresh(a).BoundingBox(1); Ty = Thresh(a).BoundingBox(2);
%         bx = blueStats(b).BoundingBox(1); by = blueStats(b).BoundingBox(2);
%         dist = ((Tx-bx)^2+(Ty-by)^2)^.5 ;
%         if (dist < 20)
%             ThreshKeep = [ThreshKeep; a];
%             dist;
%             break
%         end
%     end
% end

    

for k = 1 : length(Thresh)
    
  BBblue = Thresh(k).BoundingBox;
  aRatio = BBblue(3)/BBblue(4);
  
  %% CLASSIFY

testingFeatures = zeros(1, hogFeatureSize, 'single');
imgTest = imcrop(frame,BBblue);
%imshow(imgTest)
cropTest = imresize(imgTest,[64 64]);
%imshow(cropTest)
GTest = rgb2gray(cropTest);

% Apply pre-processing steps for testing hog data
BTest = imbinarize(GTest);
testingFeatures(1,:) = extractHOGFeatures(BTest, 'CellSize', cellSize);
Sizetest = size(testingFeatures);
testingLabels = zeros(Sizetest(1),1);
% index = (testingLabels == 1);
% testingLabels(index) = 0;
% testingLabels = testingLabels';


% Make class predictions using the test features.
predictedLabels = predict(classifierB, testingFeatures);
[p,score] = predict(classifierB, testingFeatures);
B = score;
[maxi,col] = max(B);
% Tabulate the results using a confusion matrix.
confMat = confusionmat(testingLabels, predictedLabels);


%SOME THRESHOLDING BEFORE SVM
rr = cropTest(:,:,1);
gg = cropTest(:,:,2);
bb = cropTest(:,:,3);
meanR = mean2(rr);
meanG = mean2(gg);
meanB = mean2(bb);
%USE TO VIEW SVM RESULTS
%DisplayConfusionMatrix(confMat); 
if (bikeprev == n-1 && score(2) > -.22 && ((Thresh(k).BoundingBox(1)-bikeprevx)^2 ...
        +(Thresh(k).BoundingBox(2)-bikeprevy)^2)^.5 < 45)
    col = 2;
end
if (col == 1)
    'COL1';
    if (Thresh(k).Area < 200 || Thresh(k).BoundingBox(2) < 65 || meanB< meanG || (Thresh(k).BoundingBox(2) < 114 && ...
            Thresh(k).Area > 700) || abs(meanG-meanB)<7.2 || Thresh(k).Eccentricity>.681 || meanR>118) %meanG-meanB maybe lower, dunno about meanR>
        score = -10;
    end
        'Found Arrow Sign';
        if (aRatio >.67 && aRatio < 1.44 && abs(meanG-meanB)>3)
        if (max(score) > -.182)
            %B = score
            
         Pinsert = imresize(arrow,[BBblue(3)+10 BBblue(4)+10]);
        Psize = size(Pinsert);
        startRow = int16(BBblue(2)-10);
         if (startRow < 0)
            startRow = 1;
        end
        startColumn = int16(BBblue(1)+BBblue(3)+1);
         if (startColumn < 0)
            startColumn = int16(BBblue(1)+BBblue(3)+1);
        end
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,1) = Pinsert(:,:,1);%INSERT PIC IN RIGHT POSITION
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,2) = Pinsert(:,:,2);
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,3) = Pinsert(:,:,3);
        imshow(frame)
        tx = double(startColumn)-Psize(2)-20 ; ty = double(startRow+Psize(1))+20;
        message = 'F';
        k;
        B = score;
        %tex = text(tx,ty,'FOLLOW DIRECTION','Color','y','FontSize',15);
          plotBox = [plotBox; BBblue];
%       textTrack = [textTrack ; tx ty];
%       messageTrack = [messageTrack; message];
        end
        end

end


if(col == 2)
    
    'bcyc';
    if (Thresh(k).BoundingBox(2) < 70 && Thresh(k).Area > 100 || Thresh(k).BoundingBox(2) < 136 && Thresh(k).Area > 350 ...
        ||Thresh(k).BoundingBox(2) < 153 && Thresh(k).Area > 700||  (Thresh(k).BoundingBox(1)+Thresh(k).BoundingBox(3))>1625 ...
        || (Thresh(k).BoundingBox(2)<215 && Thresh(k).BoundingBox(1) <420) || (Thresh(k).BoundingBox(2)>250 &&Thresh(k).Area < 800)...
        ||(Thresh(k).BoundingBox(1)<75 && Thresh(k).BoundingBox(2) <215) || (Thresh(k).BoundingBox(1)>1400 && Thresh(k).BoundingBox(2) <200) ... 
        ||(Thresh(k).BoundingBox(1)<145 && Thresh(k).BoundingBox(2) <2230))
        score = -10;
    end
        %imshow(ident3)
        if (max(score) > -.2)
            if (aRatio >.67 && aRatio < 1.44 && Thresh(k).Area > 350 && meanB > meanG && meanB > 69 ... %was meanB >60
                    && abs(meanG-meanB)>5.15 && Thresh(k).Eccentricity<.67)
        'Found Bike Sign';
        byc = cropTest;
        bar = aRatio;
        
        
        Pinsert = imresize(bike,[50 50]);
        Psize = size(Pinsert);
        startRow = int16(BBblue(2)-10);
        if (startRow < 0)
            startRow = 1;
        end
        startColumn = int16(BBblue(1)-50-1);
        if (startColumn < 0)
            startColumn = int16(BBblue(1)+BBblue(3)+1);
        end
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,1) = Pinsert(:,:,1);
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,2) = Pinsert(:,:,2);
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,3) = Pinsert(:,:,3);
        imshow(frame)
        tx = double(startColumn)-Psize(2)-20 ; ty = double(startRow+Psize(1))+20;
        message = 'P';
        k;
        B = score;
        bikeprev = n;
        bikeprevx = Thresh(k).BoundingBox(1);
        bikeprevy = Thresh(k).BoundingBox(2);
       % tex = text(tx,ty,'PARKING','Color','y','FontSize',15);
          plotBox = [plotBox; BBblue];
%       textTrack = [textTrack ; tx ty];
%       messageTrack = [messageTrack; message];
        %a(startrow:startrow+size(b,1)-1,startcol:startcol+size(b,2)-1) = b;
            end
        end
end

if (col == 3)
    'PP';
    PPsc = score(3);
    if (Thresh(k).BoundingBox(1)<3 || Thresh(k).Eccentricity > .9 || Thresh(k).BoundingBox(2)<3 || ...
            (Thresh(k).Area > 1000 && Thresh(k).BoundingBox(2)<80) || aRatio > 1.02||(Thresh(k).BoundingBox(2)...
            > 600 &&Thresh(k).Area < 1500) || meanG > meanB || BBblue(1)<390 && BBblue(2)<90 ... 
            ||BBblue(2)<250 && Thresh(k).Area > 7000 || abs(meanG-meanB)<4) %First one 2nd line was 320, aratio was 1.19
        score = -10;
    end
    if (PPsc > -.23 && pprev == n-1 && ((Thresh(k).BoundingBox(1)-pprevx)^2 ...
        +(Thresh(k).BoundingBox(2)-pprev)^2)^.5 < 45)
    score = 0;
    end
        if (max(score) > -.3)
        'Found Park Sign';
        
        k;
        B = score;
        Pinsert = imresize(park,[BBblue(4) BBblue(3)]);
        Psize = size(Pinsert);
        startRow = int16(BBblue(2)+1);
        startColumn = int16(BBblue(1)-BBblue(3)-1);
        if (startRow < 0)
            startRow = 1;
        end
        if (startColumn < 0)
            startColumn = int16(BBblue(1)+BBblue(3)+1);
        end
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,1) = Pinsert(:,:,1);
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,2) = Pinsert(:,:,2);
        frame(startRow:startRow+Psize(1)-1, startColumn:startColumn+Psize(2)-1,3) = Pinsert(:,:,3);
        imshow(frame)
        tx = double(startColumn)-Psize(2)-20 ; ty = double(startRow+Psize(1))+20;
        message = 'C';
       % tex = text(tx,ty,'CYCLEWAY','Color','y','FontSize',15);
          plotBox = [plotBox; BBblue];
          pprev = n; 
          pprevx = BBblue(1);
          pprevy = BBblue(2);
%          textTrack = [textTrack ; tx ty];
%          messageTrack = [messageTrack; message];
        end
end 
end
%end
%Plot all bounding boxes in blue
imshow(frameO)
hold on
bsize = size(plotBox);
for a = 1:bsize(1)
    BBblue = plotBox(a,:);
    rectangle('Position', BBblue,'EdgeColor','r','LineWidth',2);
    Counte = Counte+1;
end
title(n)

%RED BOXES
%Plot all bounding boxes in red
colPrev = 10;
rsize = size(plotBoxR);
for a = 1:rsize(1)
    BBred = plotBoxR(a,:);
    rectangle('Position', BBred,'EdgeColor','y','LineWidth',1);
    Counte = Counte+1;
    colPrev = colR;
end
% if(foundRed == 1)
%     SizeR = size(mserStatsRed);
%     if (SizeR(2) > 0)
%         for k = 1 : length(mserStatsRed)
%             BB = mserStatsRed(k).BoundingBox;
%             rectangle('Position', BB,'EdgeColor','r','LineWidth',2);
%         end
%     end
%     mserStatsRed = [];
%     foundRed = 0;
% end




%% WRITE FRAME TO VIDEO
frameGrab = getframe(gca);
BoxedSigns = frame2im(frameGrab);
writeVideo(RedClass,BoxedSigns) %Add frame to video
if(test == 1)
cla
%close all
end
n
title(n)
end
close(RedClass);
close(A)
Counte
%figure
%imshow(BoxedSigns)
% 
% 
figure
imshow(frameO)
for g = 1:length(mserStats)
    aa = mserStats(g).BoundingBox;
    rectangle('Position', aa,'EdgeColor','y','LineWidth',2);
end


function DisplayConfusionMatrix(confMat)
% Display the confusion matrix in a formatted table.

% Convert confusion matrix into percentage form
confMat = bsxfun(@rdivide,confMat,sum(confMat,2));

digits = '0':'0';
colHeadings = arrayfun(@(x)sprintf('%d',x),0:2,'UniformOutput',false);
format = repmat('%-9s',1,11);
header = sprintf(format,'digit  |',colHeadings{:});
fprintf('\n%s\n%s\n',header,repmat('-',size(header)));
for idx = 1:numel(digits)
    fprintf('%-9s',   [digits(idx) '      |']);
    fprintf('%-9.2f', confMat(idx,:));
    fprintf('\n')
end
end

