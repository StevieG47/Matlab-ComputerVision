clc; clear all
vid = VideoReader('../input/Dataset/simple.avi');
frame = read(vid,1);

Boxes = [];
cd ../input/Dataset/vehicles % go to vehicles directory
%cd test
%cd GTI_MiddleClose % go to directory
%cd GTI_RIGHT
%cd KITTI_extracted

%files.filename = ls('*png'); %all files with .png type
%names = files.filename; %names of files
lab = cell(1,419);
folder = dir('GTI_MiddleClose');
cd 'GTI_MiddleClose'
%size(frame) = [#rows #columns #channels]
for j = 4:size(folder,1) %iterate through files
       % curr_name = names(j,:);
        curr_name = folder(j).name;
        img = imread(curr_name); %current image were on
        %figure
        %imshow(img)
       % title(curr_name)
        Si = size(img); %get size
        Box = [1 1 Si(2) Si(1)]; %make bounding box entire size of image
        Boxes = [Boxes ; Box]; %adding bounding box to Boxes array
        
       % path = 'C:\Users\lekua47\Downloads\Dataset\vehicles\GTI_MiddleClose\';
       % path = 'C:\Users\Steve\Documents\Grad School\Perception\Project3\Part1\Dataset\vehicles\GTI_MiddleClose\'; %Give Path
       path = '../GTI_Right'; 
       newlabel = strcat(path,curr_name); %finish path with file name at the end
        lab{j-3} = newlabel; %Add full label name to lab
        j % view how far we are
        %cla
end


%DO SAME THING WITH GTI RIGHT FOLDER, ADD TO SAME BOXES and LAB arrays
% cd .. %directory up
% cd GTI_Right
% %cd KITTI_extracted
% files.filename = ls('*png');
% names = files.filename;
% %size(frame) = [#rows #columns #channels]
% for k = 1:size(names,1)
%         curr_name = names(k,:);
%         img = imread(curr_name);
%         %figure
%         %imshow(img)
%        % title(curr_name)
%         Si = size(img);
%         Box = [1 1 Si(2) Si(1)];
%         Boxes = [Boxes ; Box];
%         
%         %path = 'C:\Users\lekua47\Downloads\Dataset\vehicles\test\';
%         path = 'C:\Users\Steve\Documents\Grad School\Perception\Project3\Part1\Dataset\vehicles\GTI_Right\';
%         newlabel = strcat(path,curr_name);
%         lab{j+k} = newlabel;
%         j+k
%         %cla
% end
cd ../.. %Go back up
lab = lab';
%TABLE FOR CASCADE DETECTOR
Table = table(lab,Boxes); %Make the table with names, boxes

close all
%positiveFolder = fullfile('vehicles\GTI_Right')
%trainingImageLabeler(positiveFolder)



negativeFolder = fullfile('non-vehicles/GTI'); %Negative images
negativeImages = imageDatastore(negativeFolder);

sizeTable = size(Table);
trainCascadeObjectDetector('Example.xml',Table, ...
  negativeImages,'FalseAlarmRate',0.1,'NumCascadeStages',5,'NegativeSamplesFactor',2)%;,'FeatureType','Haar');

imshow(frame)

detector = vision.CascadeObjectDetector('Example.xml');
bbox = step(detector,frame);
goodBoxes = []; %good boxes to be used, we will filter out bad bounding boxes
for i = 1:size(bbox,1)
    if (bbox(i,2) > 300 && bbox(i,2) <430 && bbox(i,1) > 90) %no cars cant be in the sky so if less than 300 no good
        goodBoxes = [goodBoxes ; bbox(i,:)];
    end
 end
detectedImg = insertObjectAnnotation(frame,'rectangle',bbox,'Car'); %put bounding boxes/label onto image
figure; imshow(detectedImg);


