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

badRun = 0;
Rpos = [1 0 0; 0 1 0; 0 0 1;];
tpos = [0 0 0];
Rpos2 = Rpos;
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

   

    %GET INTRINSIC MATRIX K
     K = [fx 0  0;  %K = [fx 0 0; s fy 0; cx cy 0;]
          0  fy 0;
         cx  cy 1];
     K1 = K;
     K = K';
    %GET ESSENTIAL MATRIX
    %E = K'^T . F . K, K is intrinsic camera parameters
    [FFF,inliersIndex] = estimateFundamentalMatrix(matchedPoints1, matchedPoints2,'Method','RANSAC','NumTrials',2000,'DistanceThreshold',1e-3);
    m1X = matchedPoints1.Location(:,1);
    m1Y = matchedPoints1.Location(:,2);
    inliers1 = [m1X(inliersIndex) m1Y(inliersIndex)];
    
   m2X = matchedPoints2.Location(:,1);
   m2Y = matchedPoints2.Location(:,2);
   inliers2 = [m2X(inliersIndex) m2Y(inliersIndex)];
    
    
    
    % E = K'*Fund*K;
   % E = K' * Fund * K;
    Ess = K'*FFF*K;
    %SVD OF ESSENTIAL MATRIX
    [U1, sig, V1] = svd(Ess);
    W = [0 -1 0;
         1  0 0;
         0  0 1;];
    Z = [0 1 0;
        -1 0 0;
         0 0 0;];
     %val = (sig(1,1)+sig(2,2))/2;
     sig1 = [1 0 0; 0 1 0; 0 0 0];
     Ess2 = U1*sig1*V1'; %WAS V1'
     
     
   
 
    %GET ROTATION AND TRANSLATION
    [U2,sig2,V2] = svd(Ess2);
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
   




    %USE MATLAB TO GET FUDNAMENTAL, TAKE DIFFERENCE WITH COMPUTED
    %Fmatlab = estimateFundamentalMatrix(fMatrixPoints,fMatrixPoints2,'Method','Norm8Point');
    %Fund;
    %xprime'*Fund*x;
   % xprime'*Fmatlab*x;
   % Diff = F-Fmatlab;

    %See what matlab gets for R and t
    cameraParams = cameraParameters('IntrinsicMatrix',K);
    cameraParams1 = cameraParameters('IntrinsicMatrix',K1);
   
    [FFF,inliersIndex] = estimateFundamentalMatrix(matchedPoints1,...
    matchedPoints2,'Method','RANSAC',...
    'NumTrials',2000,'DistanceThreshold',1e-4);
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
    %EEE = K'*FFF*K;
    path  = [path  ; .05*tpos ];
    path2 = [path2 ; .05*tpos2];
    path3 = [path3 ; .05*tpos3];
    path4 = [path4 ; .05*tpos4];
    
     
    Rgood;
    tGood;
    
    %%%%%%%%%%%%%%%%%%%---estimateF function with custom Rot, t---%%%%%%%%%%%%%%%%%%%%%%%%
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

