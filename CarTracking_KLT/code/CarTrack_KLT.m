clc
close all
% READ FRAME FROM VIDEO
vid = VideoReader('../input/Dataset/simple.avi');

% WRITE VIDEO
CarDetect = VideoWriter('Car-Track44');
CarDetect.FrameRate = 30;
open(CarDetect);    
firstRun = 0
   
videoFileReader = vision.VideoFileReader('../input/Dataset/simple.avi');
videoFrame      = step(videoFileReader);
%cd BetterXML

for n = 1:1
frame = read(vid,n);

% CREATE DETECTOR BASEDON XML
detector = vision.CascadeObjectDetector('Best.xml');
%detector = vision.CascadeObjectDetector('prettygood_05_6_3.xml');
bbox = step(detector,frame);


% FILTER OUT BAD REGIONS
goodBoxes = [];
for i = 1:size(bbox,1)
    if (bbox(i,2) > 300 && bbox(i,2) <395 && bbox(i,1) > 90 && bbox(i,1) < 600 && ...
            bbox(i,3)*bbox(i,4) <9200 && bbox(i,3)*bbox(i,4) > 1300 )
        goodBoxes = [goodBoxes ; bbox(i,:)];
    end
end

% REMOVE BOXES WITHIN EACH OTHER
% Compare every box with each other. If any are within another remove the
% one within
track = [];
sizeB = size(goodBoxes);
rowsgB = size(1);
for a = 1:rowsgB
    for b = 1:rowsgB
        BB1 = goodBoxes(b,:);
        BB2 = goodBoxes(a,:);
        if (BB2(1) < BB1(1) && BB1(1) < BB2(1)+BB2(3)&& BB2(2) < BB1(2) && BB1(2) < BB2(2)+BB2(4))% if x wihtin x and y within y
            track = [track; b];
        end
    end
end
goodBoxes(track,:) = []; % Delete regions that were within others

% Get variables for the bounding boxes that made the cuts
% check size until finding a better way to do it
if size(goodBoxes,1) > 0 %1ST BOUNDING BOX
carBox = goodBoxes(1,:) ;%First bounding box for car.
    points = detectMinEigenFeatures(rgb2gray(videoFrame), 'ROI', carBox); %Get points of interest, good points in a bounding box
    bboxPoints = bbox2points(carBox(1, :));
    pointTracker = vision.PointTracker('MaxBidirectionalError', 2); %Create a point tracker and enable the bidirectional error constraint to make it more robust in the presence of noise and clutter.
    figure, imshow(videoFrame), hold on, title('Detected features');
    plot(points);
    %Initialize the tracker with the initial point locations and the initial video frame.
    points = points.Location;
    initialize(pointTracker, points, videoFrame);
    oldPoints = points
end
if size(goodBoxes,1) > 1 % 2ND BOUNDING BOX
    carBox2 = goodBoxes(2,:) ; % bounding box for another
    points2 = detectMinEigenFeatures(rgb2gray(videoFrame), 'ROI', carBox2); % Get points of interest, good points in a bounding box
    bboxPoints2 = bbox2points(carBox2(1, :));
    pointTracker2 = vision.PointTracker('MaxBidirectionalError', 2); % Create a point tracker and enable the bidirectional error constraint to make it more robust in the presence of noise and clutter.
    plot(points2);
    % Initialize the tracker with the initial point locations and the initial video frame.
    points2 = points2.Location;
    initialize(pointTracker2, points2, videoFrame);
    oldPoints2 = points2
end
if size(goodBoxes,1) > 2
    carBox3 = goodBoxes(3,:);   
    points3 = detectMinEigenFeatures(rgb2gray(videoFrame), 'ROI', carBox3); % Get points of interest, good points in a bounding box
    bboxPoints3 = bbox2points(carBox3(1, :));
    pointTracker3 = vision.PointTracker('MaxBidirectionalError', 3); % Create a point tracker and enable the bidirectional error constraint to make it more robust in the presence of noise and clutter.
    plot(points3);
    % Initialize the tracker with the initial point locations and the initial video frame.
    points3 = points3.Location;
    initialize(pointTracker3, points3, videoFrame);
    oldPoints3 = points3
end
if size(goodBoxes,1) > 3
    carBox4 = goodBoxes(4,:);
    points4 = detectMinEigenFeatures(rgb2gray(videoFrame), 'ROI', carBox4); % Get points of interest, good points in a bounding box
    bboxPoints4 = bbox2points(carBox4(1, :));
    pointTracker4 = vision.PointTracker('MaxBidirectionalError', 3); % Create a point tracker and enable the bidirectional error constraint to make it more robust in the presence of noise and clutter.
    plot(points4);
    % Initialize the tracker with the initial point locations and the initial video frame.
    points4 = points4.Location;
    initialize(pointTracker4, points4, videoFrame);
    oldPoints4 = points4
end
if size(goodBoxes,1) > 4
carBox5 = goodBoxes(5,:);
end


 count = 0;

test = 0;
test1 = 0;
test2 = 0;

 
% SHOULD DO DETECTION DONE ABOVE EVERY N FRAMES IN CASE OF NEW CARS OR JUST
% TO UPDATE
 while ~isDone(videoFileReader)
    
    % EVERY N FRAMES WE WILL DO DETECTION TO UPDATE
    N = 4; % 4 worked ok
    if rem(count,N) == 0 && count > 0 % if remainder of count/10 is zero ie every 10 frames
        bbox = step(detector,frame);
        
        

        % FILTER OUT BAD REGIONS
        goodBoxes = [];
        for i = 1:size(bbox,1)
            if (bbox(i,2) > 300 && bbox(i,2) <395 && bbox(i,1) > 90 && bbox(i,1) < 600 && ...
                    bbox(i,3)*bbox(i,4) <9200 && bbox(i,3)*bbox(i,4) > 1300 )
                goodBoxes = [goodBoxes ; bbox(i,:)];
            end
        end

        % REMOVE BOXES WITHIN EACH OTHER
        % Compare every box with each other. If any are within another remove the
        % one within
        
        track = [];
        sizeB = size(goodBoxes);
        rowsgB = size(1);
        for a = 1:rowsgB
            for b = 1:rowsgB
                BB1 = goodBoxes(b,:);
                BB2 = goodBoxes(a,:);
                if (BB2(1) < BB1(1) && BB1(1) < BB2(1)+BB2(3)&& BB2(2) < BB1(2) && BB1(2) < BB2(2)+BB2(4))% if x wihtin x and y within y
                    track = [track; b];
                end
            end
        end
        goodBoxes(track,:) = []; % Delete regions that were within others
        % View boxes
       % figure
        videoFrame = step(videoFileReader);
        detectedImg = insertObjectAnnotation(videoFrame,'rectangle',goodBoxes,'Car');
        imshow(detectedImg)
        
        % Loop through goodBoxes to match them with rectangels we already 
        % have from tracker
        for a = 1:size(goodBoxes,1)
           % if ((goodBoxes(a,1)-rect(1))^2 + (goodBoxes(a,2)-rect(2)^2)^.5 < 30) % if goodBox box has same top left location as rect (from carBox1)
             if ((goodBoxes(a,1)-rect(1))^2 + (goodBoxes(a,2)-rect(2))^2)^.5 < 15     
                 carBox = goodBoxes(a,:);
                % Do everything we did above to initialize the tracker but
                %with the updated bounding box
                points = detectMinEigenFeatures(rgb2gray(videoFrame), 'ROI', carBox); %update good points for 1st one
                bboxPoints = bbox2points(carBox(1, :));
                points = points.Location;
                release(pointTracker); %Added this to reset
                initialize(pointTracker, points, videoFrame);
                oldPoints = points;
                'Updated rect 1'
                test2 = 1;
            end
        
    
        % IF MORE THAN ONE BOUNDING BOX IN GOOD BOXES DO IT AGAIN
            % Check rect 2
            if size(goodBoxes,1) > 1
                if ((goodBoxes(a,1)-rect2(1))^2 + (goodBoxes(a,2)-rect2(2))^2)^.5 < 30 %if goodBox box has same top left location as rect (from carBox1)
                    carBox2 = goodBoxes(a,:);
                    % Do everything we did above to initialize the tracker but
                    % with the updated bounding box
                    points2 = detectMinEigenFeatures(rgb2gray(videoFrame), 'ROI', carBox2); %update good points for 1st one
                    bboxPoints2 = bbox2points(carBox2(1, :));
                    points2 = points2.Location;
                    release(pointTracker2); %Added this to reset
                    initialize(pointTracker2, points2, videoFrame);
                    oldPoints2 = points2;
                    'Updated rect 2'
                    test1 = 1;
                end
            end
            
            % IF MORE THAN TWO BOUNDING BOX IN GOOD BOXES DO IT AGAIN
            % Check rect 3
            if size(goodBoxes,1) > 2
                if ((goodBoxes(a,1)-rect3(1))^2 + (goodBoxes(a,2)-rect3(2))^2)^.5 < 30 %if goodBox box has same top left location as rect (from carBox1)
                    carBox3 = goodBoxes(a,:);
                    insertObjectAnnotation(videoFrame,'rectangle',carBox3,'Test','Color','yellow');
                    %rect
                    %Do everything we did above to initialize the tracker but
                    %with the updated bounding box
                    points3 = detectMinEigenFeatures(rgb2gray(videoFrame), 'ROI', carBox3); %update good points for 1st one
                    bboxPoints3 = bbox2points(carBox3(1, :));
                    points3 = points3.Location;
                    release(pointTracker3); %Added this to reset
                    initialize(pointTracker3, points3, videoFrame);
                    oldPoints3 = points3;
                    'Updated rect 3'
                    test = 1;
                end
            end
        end
                
        end
 
%     
%         
%         
%         
%         
%         
%         
%         
%         
    % get the next frame
    videoFrame = step(videoFileReader);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % Track the points. Note that some points may be lost.
    [points, isFound] = step(pointTracker, videoFrame);
    visiblePoints = points(isFound, :);
    oldInliers = oldPoints(isFound, :);
    
    %Again for next box
    if size(goodBoxes,1) > 1
        [points2, isFound] = step(pointTracker2, videoFrame);
        visiblePoints2 = points2(isFound, :);
        oldInliers2 = oldPoints2(isFound, :);
    end
    %Again
    if size(goodBoxes,1) >2
    [points3, isFound] = step(pointTracker3, videoFrame);
    visiblePoints3 = points3(isFound, :);
    oldInliers3 = oldPoints3(isFound, :);
    end
    
    %Again if a 4th box
    if size(goodBoxes,1) >3
    [points4, isFound] = step(pointTracker4, videoFrame);
    visiblePoints4 = points4(isFound, :);
    oldInliers4 = oldPoints4(isFound, :);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     if size(visiblePoints, 1) >= 2 % need at least 2 points

        % Estimate the geometric transformation between the old points
        % and the new points and eliminate outliers
        [xform, oldInliers, visiblePoints] = estimateGeometricTransform(...
            oldInliers, visiblePoints, 'similarity', 'MaxDistance', 4);

        % Apply the transformation to the bounding box points
        bboxPoints = transformPointsForward(xform, bboxPoints);

        % Insert a bounding box around the object being tracked
        bboxPolygon = reshape(bboxPoints', 1, []);
        %videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, ...
         %   'LineWidth', 2, 'Color', 'blue');
        
        %Insert label and box
        %bboxPolygon is square but has rotation, we dont want any rotation
        %on cars so get xpoints, ypoints and make a rectangle
        xPoints = [bboxPolygon(1) bboxPolygon(3) bboxPolygon(5) bboxPolygon(7)];
        yPoints = [bboxPolygon(2) bboxPolygon(4) bboxPolygon(6) bboxPolygon(8)];
        xmin = min(xPoints);
        ymin = min(yPoints);
        xmax = max(xPoints);
        ymax = max(yPoints);
        rect = [xmin ymin xmax-xmin ymax-ymin];
        %if test2 == 1
         videoFrame = insertObjectAnnotation(videoFrame,'rectangle',rect,'Car','Color','green');
       test2 = 0;
        %end
         % Display tracked points
        videoFrame = insertMarker(videoFrame, visiblePoints, '+', ...
            'Color', 'white');
    % Reset the points
        oldPoints = visiblePoints;
        setPoints(pointTracker, oldPoints);
     end
%         
%     
     %2nd one Now
     if size(visiblePoints2, 1) >= 2 % need at least 2 points

        % Estimate the geometric transformation between the old points
        % and the new points and eliminate outliers
        [xform2, oldInliers2, visiblePoints2] = estimateGeometricTransform(...
            oldInliers2, visiblePoints2, 'similarity', 'MaxDistance', 4);

        % Apply the transformation to the bounding box points
        bboxPoints2 = transformPointsForward(xform2, bboxPoints2);

        % Insert a bounding box around the object being tracked
        bboxPolygon2 = reshape(bboxPoints2', 1, []);
       % videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon2, ...
        %    'LineWidth', 2, 'Color', 'blue');
        
        %Insert label and box
        %bboxPolygon is square but has rotation, we dont want any rotation
        %on cars so get xpoints, ypoints and make a rectangle
        xPoints2 = [bboxPolygon2(1) bboxPolygon2(3) bboxPolygon2(5) bboxPolygon2(7)];
        yPoints2 = [bboxPolygon2(2) bboxPolygon2(4) bboxPolygon2(6) bboxPolygon2(8)];
        xmin2 = min(xPoints2);
        ymin2 = min(yPoints2);
        xmax2 = max(xPoints2);
        ymax2 = max(yPoints2);
        rect2 = [xmin2 ymin2 xmax2-xmin2 ymax2-ymin2];
        %if (test1 ==1)
        videoFrame = insertObjectAnnotation(videoFrame,'rectangle',rect2,'Car','Color','cyan');
        %test1 = 0;
       % end
        % Display tracked points
        videoFrame = insertMarker(videoFrame, visiblePoints2, '+', ...
            'Color', 'white');
    % Reset the points
        oldPoints2 = visiblePoints2;
        setPoints(pointTracker2, oldPoints2);
     end
     
     
     %3rd one
     if size(visiblePoints3, 1) >= 2 % need at least 2 points

        % Estimate the geometric transformation between the old points
        % and the new points and eliminate outliers
        [xform3, oldInliers3, visiblePoints3] = estimateGeometricTransform(...
            oldInliers3, visiblePoints3, 'similarity', 'MaxDistance', 4);

        % Apply the transformation to the bounding box points
        bboxPoints3 = transformPointsForward(xform3, bboxPoints3);

        % Insert a bounding box around the object being tracked
        bboxPolygon3 = reshape(bboxPoints3', 1, []);
        %videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, ...
         %   'LineWidth', 2, 'Color', 'blue');
        
        %Insert label and box
        %bboxPolygon is square but has rotation, we dont want any rotation
        %on cars so get xpoints, ypoints and make a rectangle
        xPoints3 = [bboxPolygon3(1) bboxPolygon3(3) bboxPolygon3(5) bboxPolygon3(7)];
        yPoints3 = [bboxPolygon3(2) bboxPolygon3(4) bboxPolygon3(6) bboxPolygon3(8)];
        xmin3 = min(xPoints3);
        ymin3 = min(yPoints3);
        xmax3 = max(xPoints3);
        ymax3 = max(yPoints3);
        rect3 = [xmin3 ymin3 xmax3-xmin3 ymax3-ymin3];
       % if (test == 1)
        videoFrame = insertObjectAnnotation(videoFrame,'rectangle',rect3,'Car','Color','yellow');
        %test = 0;
        %end
        % Display tracked points
        videoFrame = insertMarker(videoFrame, visiblePoints3, '+', ...
            'Color', 'white');
    % Reset the points
        oldPoints3 = visiblePoints3;
        setPoints(pointTracker3, oldPoints3);
     end
     
      %4th one
     if size(visiblePoints4, 1) >= 2 % need at least 2 points

        % Estimate the geometric transformation between the old points
        % and the new points and eliminate outliers
        [xform4, oldInliers4, visiblePoints4] = estimateGeometricTransform(...
            oldInliers4, visiblePoints4, 'similarity', 'MaxDistance', 4);

        % Apply the transformation to the bounding box points
        bboxPoints4 = transformPointsForward(xform4, bboxPoints4);

        % Insert a bounding box around the object being tracked
        bboxPolygon4 = reshape(bboxPoints4', 1, []);
        %videoFrame = insertShape(videoFrame, 'Polygon', bboxPolygon, ...
         %   'LineWidth', 2, 'Color', 'blue');
        
        %Insert label and box
        %bboxPolygon is square but has rotation, we dont want any rotation
        %on cars so get xpoints, ypoints and make a rectangle
        xPoints4 = [bboxPolygon4(1) bboxPolygon4(3) bboxPolygon4(5) bboxPolygon4(7)];
        yPoints4 = [bboxPolygon4(2) bboxPolygon4(4) bboxPolygon4(6) bboxPolygon4(8)];
        xmin4 = min(xPoints4);
        ymin4 = min(yPoints4);
        xmax4 = max(xPoints4);
        ymax4 = max(yPoints4);
        rect4 = [xmin4 ymin4 xmax4-xmin4 ymax4-ymin4];
        videoFrame = insertObjectAnnotation(videoFrame,'rectangle',rect4,'Car','Color','red');
        % Display tracked points
        videoFrame = insertMarker(videoFrame, visiblePoints4, '+', ...
            'Color', 'white');
    % Reset the points
        oldPoints4 = visiblePoints4;
        setPoints(pointTracker4, oldPoints4);
     end
%      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     %figure
     %imshow(videoFrame)
     %cla
     writeVideo(CarDetect,videoFrame) %Add frame to video, (vid,frame)
   % Display the annotated video frame using the video player object
    %step(videoPlayer, videoFrame);
    count;
    count = count+1

end

% Clean up
release(videoFileReader);

%release(videoPlayer);
close(CarDetect)
release(pointTracker);


cd ..
end