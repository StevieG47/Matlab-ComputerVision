clc
clear all
close all
warning('off','images:initSize:adjustingMag');

%You are given frames from a driving car, scripts to extract intrinsic parameters
%and the output should be a plot for the trajectory of camera 
turn = 0;
curX = 0;
curY = 0;
curZ = 0;
fig = 1;
path = [];
path2 = [];
path3 = [];
path4 = [];
%C1 = [0 0 0];
%C2 = C1;
badRun = 0;
Rpos = [1 0 0; 0 1 0; 0 0 1;];
Rpos2 = Rpos;
tpos = [0 0 0];
tpos2 = tpos;
tpos3 = tpos;
Rpos3 = Rpos;
tpos4 = tpos;
Rpos4 = Rpos;
NewVid = VideoWriter('DrivingCar');
Plot = VideoWriter('PlotTrajectory');
Plot.FrameRate = 10;
open(NewVid);
heading3 = 0;
open(Plot);
%BAYER TO RGB
%use demosaic function with GBRG alignment
%RGB = demosaic(I,sensorAlignment) converts encoded image I,sensorAlignment is Bayer pattern
cd ../input
for i = 600:1:1000
    
    
    cd Oxford_dataset/stereo/centre
    files.filename = ls('*png'); %all files with .png type
    picNames = files.filename;
    %i = 1;
    currentName = picNames(i,:);
    image = imread(currentName); %read bayer image
    %imshow(image) show bayer image
    colorIm = demosaic(image,'gbrg'); %convert to rgb image
    %figure
    %imshow(colorIm) %show colored image

    %get image right after
    nextName = picNames(i+1,:);
    image = imread(nextName);
    colorImNext = demosaic(image,'gbrg');


    %EXTRACT CAMERA PARAMETERS
    cd ../..
    % top line of stereo narrow left.txt gives fx, fy, cx, cy
    [fx, fy, cx, cy, G_camera_image, LUT] = ReadCameraModel('./stereo/centre','./model'); 


    %UNDISTORT THE CURRENT FRAME AND THE NEXT FRAME
    %Use UndistortImage.m with rgb image and LUT we just found to get undistorted image
    frame = UndistortImage(colorIm,LUT); 
    frameNext = UndistortImage(colorImNext,LUT);

    %GAUSSIAN FILTER FRAME
    G = fspecial('gaussian', [5,5],2);
    frame = imfilter(frame,G,'same');%gaussian filter
    frameNext = imfilter(frameNext,G,'same');%gaussian filter

    %SIZE OF FRAME
    widthF = size(frame,2);
    lengthF= size(frame,1);

    %ADD FRAME TO VIDEO
    writeVideo(NewVid,frame) %Add frame to video
    
    %FIND POINT CORRESPONDENCES
    %Corners? uh features?
    %TRY SURF FEATURES
    frameGray = rgb2gray(frame);
    frameNextGray = rgb2gray(frameNext);
    surf1 = detectSURFFeatures(frameGray);
    surf2 = detectSURFFeatures(frameNextGray);

    %Plot image with points
    %imshow(frame); hold on;
    %plot(surf1.selectStrongest(20))
    %figure
    %imshow(frameNext); hold on;
    %plot(surf2.selectStrongest(20))

    %EXTRACT FEATURES FROM IMAGE
    [f1, vpts1] = extractFeatures(frameGray, surf1);
    [f2, vpts2] = extractFeatures(frameNextGray, surf2);

    %MATCH FEATURES BETWEEN IMAGES
    indexPairs = matchFeatures(f1, f2, 'MaxRatio',.3);
    matchedPoints1 = vpts1(indexPairs(:, 1));
    matchedPoints2 = vpts2(indexPairs(:, 2));
    
    %SHOW MATCHES FEATURES
    %figure;
   % showMatchedFeatures(frame,frameNext,matchedPoints1,matchedPoints2);
   % legend('unique matched points 1','unique matched points 2');

    %matchedPoints1.Location and %matchedPoints2.Location are matrices of nx2
    %points with x,y position. Each index should be matched point
    %correspondences. So matchedPoints1.Location(1,:) and
    %matchedPoints2.Location(1,:) are the first points in each frame, they
    %should be pretty close since they are matched points

    %PICK 8 random points from the matched points. Make sure the are matched
    %(some of the matches get fucked up and distance between them is large)
    %Use these 8 good points for fundamental matrix.
    %Also point less then 780 since above that y value is just the car

    %For our random index, check if matched points are close (some of the
    %correspondences will mess up and "matched" points will be across screen
    %Also check if y value less than 780 since the front of the car is in
    %the frame there wait not dont do that
    %If it passes assign x,y points to be used in Fundamental Matrix caluclation
    %To ensure 8 points, loop and add good points to fMatrixPoints until we have added 8 points
    fMatrixPoints = [];
    fMatrixPoints2 = [];
    count = 0;
    foundPoints = 0;
    allGoodPoints1 = [];
    allGoodPoints2 = [];
    thresh1 = 3;
    while length(fMatrixPoints) < 8
        for k = 1:length(matchedPoints1.Location)
            if (((matchedPoints1.Location(k,1) - matchedPoints2.Location(k,1))^2 + (matchedPoints1.Location(k,2)-matchedPoints2.Location(k,2))^2)^.5 < thresh1)% ...
                 %   && matchedPoints1.Location(randIndex1,2) < 780)
                x1 = matchedPoints1.Location(k,1);
                y1 = matchedPoints1.Location(k,2);
                x2 = matchedPoints2.Location(k,1);
                y2 = matchedPoints2.Location(k,2);
                fMatrixPoints = [fMatrixPoints ; [x1,y1]];
                fMatrixPoints2 = [fMatrixPoints2 ; [x2,y2]];
                ((matchedPoints1.Location(k,1) - matchedPoints2.Location(k,1))^2 + (matchedPoints1.Location(k,2)-matchedPoints2.Location(k,2))^2)^.5;
            end
        end
        thresh1 = thresh1 + 5;
        count = count+1;

    end

    if foundPoints == 0
%     close all
%     figure
%     imshow(frame); hold on
%     scatter(fMatrixPoints(:,1),fMatrixPoints(:,2),90,'r','filled')
%     scatter(fMatrixPoints2(:,1),fMatrixPoints2(:,2),80,'g','+')

    %GET FUNDAMENTAL MATRIX
    %function will be getFundamental(matrix1,matrix2) where matrix 1 has x,y
    %points of frame1 and matrix2 has x,y, points of frame2
    %ESTIMATE THE FUNDAMENTAL MATRIX

    %For fundamental matrix need 8 point matchings so 8 points from first frame
    %then 8 corresponding points in the next frame
    %In homography each point correspondence contributed 2 constraints,  but
    %for fundaental matrix we need 8 since each only contributes 1 constraint
    %bc scalar equation
    %Fundamental matrix is x'T F x = 0
    %With equation for a pair of points, 
    %Then have a homogenous system we need solution to: Ah = 0
    %Do SVD, A = UDV^T
    %Procedure: Construct matrix A, Find SVD, F is last column of V
    %[U,D,V] = svd(A), F = reshape(V(:,9),3,3)', [FU FD FV] = svd(F);, FD(3,3)=0, F = FU*FD*FV'

    %DEW IT
    % need 8 points not 4
    %Af  = 0
    %A = [x1x1' x1y1' x1  y1x1' y1y1' y1 x1' y1' 1]
    %     ..........................................
    %    [x8x8' x8y8' x8  y8x8' y8y8' y8 x8' y8' 1]


    %Points from frame1
    x1 = fMatrixPoints(1,1); y1 = fMatrixPoints(1,2);
    x2 = fMatrixPoints(2,1); y2 = fMatrixPoints(2,2);
    x3 = fMatrixPoints(3,1); y3 = fMatrixPoints(3,2);
    x4 = fMatrixPoints(4,1); y4 = fMatrixPoints(4,2);
    x5 = fMatrixPoints(5,1); y5 = fMatrixPoints(5,2);
    x6 = fMatrixPoints(6,1); y6 = fMatrixPoints(6,2);
    x7 = fMatrixPoints(7,1); y7 = fMatrixPoints(7,2);
    x8 = fMatrixPoints(8,1); y8 = fMatrixPoints(8,2);

    %Points from frame2
    x1P = fMatrixPoints2(1,1); y1P = fMatrixPoints2(1,2);
    x2P = fMatrixPoints2(2,1); y2P = fMatrixPoints2(2,2);
    x3P = fMatrixPoints2(3,1); y3P = fMatrixPoints2(3,2);
    x4P = fMatrixPoints2(4,1); y4P = fMatrixPoints2(4,2);
    x5P = fMatrixPoints2(5,1); y5P = fMatrixPoints2(5,2);
    x6P = fMatrixPoints2(6,1); y6P = fMatrixPoints2(6,2);
    x7P = fMatrixPoints2(7,1); y7P = fMatrixPoints2(7,2);
    x8P = fMatrixPoints2(8,1); y8P = fMatrixPoints2(8,2);

    %MATRIX A
    A  =[x1*x1P x1*y1P x1 y1*x1P y1*y1P y1 x1P y1P 1; ...
         x2*x2P x2*y2P x2 y2*x2P y2*y2P y2 x2P y2P 1;
         x3*x3P x3*y3P x3 y3*x3P y3*y3P y3 x3P y3P 1;
         x4*x4P x4*y4P x4 y4*x4P y4*y4P y4 x4P y4P 1;
         x5*x5P x5*y5P x5 y5*x5P y5*y5P y5 x5P y5P 1;
         x6*x6P x6*y6P x6 y6*x6P y6*y6P y6 x6P y6P 1;
         x7*x7P x7*y7P x7 y7*x7P y7*y7P y7 x7P y7P 1;
         x8*x8P x8*y8P x8 y8*x8P y8*y8P y8 x8P y8P 1;];

     A = double(A);
    %SVD OF A
    [U, D, V] = svd(A);

    %Get Fundamental Matrix'
    F = reshape(V(:,9),3,3)';
    [FU, FD, FV] = svd(F);
    FD(3,3)=0;
    Fund = FU*FD*FV;
    
    %quick test
    xprime = [fMatrixPoints2(1,:) 0]';
    x = [fMatrixPoints(1,:) 0]';
    xprime'*Fund*x; %see if this equals zero ..every time
    end

    %GET INTRINSIC MATRIX K
     K = [fx 0  0;  %K = [fx 0 0; s fy 0; cx cy 0;]
          0  fy 0;
         cx  cy 1];
    K1 = K;
    K = K';
    %GET ESSENTIAL MATRIX
    %E = K'^T . F . K, K is intrinsic camera parameters
    %[FFF,inliersIndex] = estimateFundamentalMatrix(matchedPoints1, matchedPoints2,'Method','RANSAC','NumTrials',2000,'DistanceThreshold',1e-3);
      
    
        % E = K'*Fund*K;
    E = K' * Fund * K;
    
    %SVD OF ESSENTIAL MATRIX
    [U1, sig, V1] = svd(E);
    
    W = [0 -1 0;
         1  0 0;
         0  0 1;];
    Z = [0 1 0;
        -1 0 0;
         0 0 0;];
    % val = (sig(1,1)+sig(2,2))/2;
     sig1 = [1 0 0; 0 1 0; 0 0 0];
     E2 = U1*sig1*V1; %WAS V1'
     
 
    %GET ROTATION AND TRANSLATION
    [U2,sig2,V2] = svd(E2);
    R1 = U2*W'*V2';
    R2 = U2*W*V2';
    if det(R1) < 0
        R1 = -R1;
    end
  
    if det(R2) < 0
        R2 = -R2;
    end
    Tx = U2*Z*U2';
    U2;
    %Tx = [0 -Tz Ty; Tz 0 -Tx; -Ty Tx 0;]
    t1 = [Tx(3,2) Tx(1,3) Tx(2,1)];
    t2 = -t1;
    %Car always moves forward
    if t1(3) > 0
        tGood = t1;
    end
    if t2(3) > 0
        tGood = t2;
    end
   tGood;
   
    
    %See what matlab gets 
    cameraParams = cameraParameters('IntrinsicMatrix',K);
    cameraParams1 = cameraParameters('IntrinsicMatrix',K1);
    
    [FFF,inliersIndex] = estimateFundamentalMatrix(matchedPoints1,...
    matchedPoints2,'Method','RANSAC',...
    'NumTrials',2000,'DistanceThreshold',1e-3);
    m1X = matchedPoints1.Location(:,1);
    m1Y = matchedPoints1.Location(:,2);
    inliers1 = [m1X(inliersIndex) m1Y(inliersIndex)];
    
   m2X = matchedPoints2.Location(:,1);
   m2Y = matchedPoints2.Location(:,2);
   inliers2 = [m2X(inliersIndex) m2Y(inliersIndex)];
   [Rot4,tran4] = relativeCameraPose(FFF,cameraParams,inliers1,inliers2);
  
   %Rotation matrices
   if(R1(1,1)>0 && R1(2,2)>0 && R1(3,3)>0)
       Rgood = R1;
   end
   if R2(1,1)>0 && R2(2,2) > 0 && R2(3,3) > 0
       Rgood = R2;
   end
   
   %USE MATLABS ROT TO GET R1 or r2 as good
    path  = [path  ; .05*tpos ];
    path2 = [path2 ; .05*tpos2];
    path3 = [path3 ; .05*tpos3];
    path4 = [path4 ; .05*tpos4];
    
  
       
    Rgood ;
    tGood;
    %%%%%%%%%%%%%%%%%%%---ALL MY STUFF---%%%%%%%%%%%%%%%%%%%%%%%%
    Rpos = Rgood*Rpos ;%pos is the pose of the camera
    tpos = tpos + tGood*Rpos;     
     
    
    %ALL MATLAB STUFF
    [Rot4,tran4] = relativeCameraPose(FFF,cameraParams1,inliers1,inliers2);
    Rpos4 = Rot4*Rpos4 ;%pos is the pose of the camera
    tpos4 = tpos4 + tran4*Rpos4; 
    tran4;
    Rot4;
    heading4 = acos(Rot4(1,1))*180/pi;
    
    
    
   % C1 = C1*relO + relL;
    i = i+1

cd ..
end

badRun
axisLimit = 5;
close(NewVid);

