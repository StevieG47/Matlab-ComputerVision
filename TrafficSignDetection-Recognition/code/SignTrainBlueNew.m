%% Training 1st one
clear all
park = imread('park.png');
bike = imread('bike.png');
arrow = imread('arrow.png');

%cd ./Training
cd ./Stop
folder = num2str(35,'%05i');
cd(folder)
files.filename = ls('*ppm');
names = files.filename;
for j = 30:30%size(names,1)
        curr_name = names(j,:);
        img = imread(curr_name);
        %figure
        %imshow(img)
        title(folder)
end

curr_name = names(5,:);
img = imread(curr_name);
%figure
%imshow(img)
crop = imresize(img,[64 64]);

%HOG SHIT
% Extract HOG features and HOG visualization
[hog_2x2, vis2x2] = extractHOGFeatures(crop,'CellSize',[2 2]);
[hog_4x4, vis4x4] = extractHOGFeatures(crop,'CellSize',[4 4]);
[hog_8x8, vis8x8] = extractHOGFeatures(crop,'CellSize',[8 8]);

%%  LETS DO HOG
cellSize = [2 2];
hogFeatureSize = length(hog_2x2);
% Loop over the trainingSet and extract HOG features from each image. A
% similar procedure will be used to extract features from the testSet.

%% TRAINING FOR 1st one
Size1 = size(names);
trainingFeatures1 = zeros(Size1(1), hogFeatureSize, 'single');
Rmean = [];
Gmean = [];
Bmean = [];
for i = 1:Size1(1)
    curr_name = names(i,:);
    img = imread(curr_name);
    %figure
    %imshow(img)
    crop = imresize(img,[64 64]);
    %imshow(crop)
    G = rgb2gray(crop);

    % Apply pre-processing steps
    B = imbinarize(G);
    
    trainingFeatures1(i,:) = extractHOGFeatures(B, 'CellSize', cellSize);
   % [i Size(1)]
   if (i == 23)
   bluedent1 = crop;
   end
%    r = crop (:,:,1);
%    g = crop(:,:,2);
%    b = crop(:,:,3);
%    meanr = mean2(r);
%    meang = mean2(g);
%    meanb = mean2(b);
%    Rmean = [Rmean; meanr];
%    Gmean = [Gmean; meang];
%    Bmean = [Bmean; meanb];
   
end
% rrr = mean2(Rmean)
% ggg = mean2(Gmean)
% bbb = mean2(Bmean)
train1 = i;
%TRAINING LABEL FOR THIS IS 0
Size = size(trainingFeatures1);
trainingLabels1 = zeros(Size(1),1);


%% TRAINING 2nd one
cd ..
folder = num2str(38,'%05i');
cd(folder)
files.filename = ls('*ppm');
names = files.filename;
Size2 = size(names);
trainingFeatures2 = zeros(Size2(1), hogFeatureSize,'single');
%trainingFeatures = [trainingFeatures ; trainingFeaturesAdd];
for i = 1:Size2(1)
    curr_name = names(i,:);
    img = imread(curr_name);
    %figure
    %imshow(img)
    crop = imresize(img,[64 64]);
   % imshow(crop)
    G = rgb2gray(crop);

    % Apply pre-processing steps
    B = imbinarize(G);
    
    trainingFeatures2(i,:) = extractHOGFeatures(B, 'CellSize', cellSize);
   % [i Size(1)]
   if (i == 58)
   bluedent2 = crop;
   end
end
SizeAdd2 = size(trainingFeatures2);
trainingLabels2 = ones(SizeAdd2(1),1);
%trainingLabels = vertcat(trainingLabels1, trainingLabels2);
%trainingFeatures = vertcat(trainingFeatures1,trainingFeatures2);


%% TRAINIGN FOR THIRD SIGN
cd ..
folder = num2str(45,'%05i');
cd(folder)
files.filename = ls('*ppm');
names = files.filename;
Size3 = size(names);
trainingFeatures3 = zeros(Size3(1), hogFeatureSize,'single');
%trainingFeatures = [trainingFeatures ; trainingFeaturesAdd];
for i = 1:Size3(1)
    curr_name = names(i,:);
    img = imread(curr_name);
    %figure
    %imshow(img)
    crop = imresize(img,[64 64]);
  %  imshow(crop)
    G = rgb2gray(crop);

    % Apply pre-processing steps
    B = imbinarize(G);
    
    trainingFeatures3(i,:) = extractHOGFeatures(B, 'CellSize', cellSize);
   % [i Size(1)]
   if (i == 26)
   bluedent3 = crop;
   end
end
Size3 = size(trainingFeatures3);
trainingLabels3 = ones(Size3(1),1);
index = (trainingLabels3 == 1);
trainingLabels3(index) = 2;

% Size = size(trainingFeatures3);
% trainingLabelsAdd2 = ones(Size(1),1);
% index = (trainingLabelsAdd2 == 1);
% trainingLabelsAdd2(index) = 2;

%% SVM
%cd ../..
%clc
%CREATE TRAINIGN FEATURES AND TRAINING LABELS
trainingFeatures = vertcat(trainingFeatures1,trainingFeatures2, trainingFeatures3);
trainingLabels = vertcat(trainingLabels1, trainingLabels2,trainingLabels3);

% fitcecoc uses SVM learners and a 'One-vs-One' encoding scheme.
classifierB = fitcecoc(trainingFeatures, trainingLabels);

testes = fitcecoc(trainingFeatures1,trainingLabels1);


%% GO TO TEST NOW THAT TRAINING CLASSIFIER IS DONE
cd ../../Testing
folder = num2str(38,'%05i');
cd(folder)
files.filename = ls('*ppm');
namesT = files.filename;
SizeT = size(namesT);
testingFeatures = zeros(1, hogFeatureSize, 'single');

for i = 1:1
testName = namesT(10,:);
imgTest = imread(testName);
%figure
imshow(imgTest)
cropTest = imresize(imgTest,[64 64]);
imshow(cropTest)
GTest = rgb2gray(cropTest);

% Apply pre-processing steps for testing hog data
BTest = imbinarize(GTest);
testingFeatures(i,:) = extractHOGFeatures(BTest, 'CellSize', cellSize);
end
Sizetest = size(testingFeatures);
testingLabels = zeros(Sizetest(1),1);
% index = (testingLabels == 1);
% testingLabels(index) = 0;
% testingLabels = testingLabels';


% Make class predictions using the test features.
predictedLabels = predict(classifierB, testingFeatures);
[p,score] = predict(testes, testingFeatures);
%score


[p,score] = predict(classifierB, testingFeatures);
B = score
[maxi,col] = max(B);


imshow(cropTest)
figure
if (col == 1)
imshow(bluedent1)
end
if (col == 2)
    imshow(bluedent2)
end
if(col ==3)
    imshow(bluedent3)
end


% Tabulate the results using a confusion matrix.
%confMat = confusionmat(testingLabels, predictedLabels)

%helperDisplayConfusionMatrix(confMat)
%DisplayConfusionMatrix(confMat)
cd ../..

%Find which column has highest value, corresponds to highest percentage
% confMat2 = confMat';
% [M,I] = max(confMat2);
% figure
% if (I(1) == 1)
%     imshow(ident1);
% end
% if(I(1) == 2)
%     imshow(ident2);
% end
% % if(I(1) == 3)
% %     imshow(ident3);
% % end
% 
% function DisplayConfusionMatrix(confMat)
% % Display the confusion matrix in a formatted table.
% 
% % Convert confusion matrix into percentage form
% confMat = bsxfun(@rdivide,confMat,sum(confMat,2));
% 
% digits = '0':'0';
% colHeadings = arrayfun(@(x)sprintf('%d',x),0:2,'UniformOutput',false);
% format = repmat('%-9s',1,11);
% header = sprintf(format,'digit  |',colHeadings{:});
% fprintf('\n%s\n%s\n',header,repmat('-',size(header)));
% for idx = 1:numel(digits)
%     fprintf('%-9s',   [digits(idx) '      |']);
%     fprintf('%-9.2f', confMat(idx,:));
%     fprintf('\n')
% end
% end
% 
