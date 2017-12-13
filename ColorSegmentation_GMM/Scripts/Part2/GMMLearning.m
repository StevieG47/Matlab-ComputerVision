clear all
clc

imagefiles = dir('../../Images/TrainingSet/CroppedBuoys/Red/*.jpg');
cd('../../Images/TrainingSet/CroppedBuoys/Red')

nfiles = length(imagefiles);    % Number of files found
C = cell(nfiles,1);
Samples = [];
for i=1:nfiles%for every cropped image
   currentfilename = imagefiles(i).name;
   frame = imread(currentfilename);
   
   %Get Channels
redChannel = frame(:, :, 1);
greenChannel = frame(:, :, 2);
blueChannel = frame(:, :, 3);

%Get arrays of every Channel value
[a b] = size(redChannel);%get dimensions of color channels
pixelR = [];%matrices for color channels. Basically want a straight array of values 0-255 for each pixel, for each color
pixelG = [];
pixelB = [];
for i = 1:a
    for j = 1:b
        pixelR = [pixelR redChannel(i,j)];%add value to color matrix
        pixelG = [pixelG greenChannel(i,j)];
        pixelB = [pixelB blueChannel(i,j)];
    end
end
pixelR = pixelR';%transpose to make 1 column
pixelG = pixelG';
pixelB = pixelB';
Samp = [pixelR pixelG pixelB];
Samples = [Samples ; Samp];
end


%HISTOGRAMS FOR EACH CHANNEL
[yRed, x] = imhist(Samples(:,1));
stem(x, yRed,'r');%AFTER INSPECTION, JUST 1 GAUSSIAN FOR RED CHANNEL, RED BUOY
title('Red Buoy: red Channel')

[yGreen, x] = imhist(Samples(:,2));
figure
stem(x,yGreen,'g')%AFTER INSPECTION, 2 GAUSSIANS FOR GREEN CHANNEL, RED BUOY
title('Red Buoy: green Channel')

[yBlue, x] = imhist(Samples(:,3));%2 GUASSIANS FOR BLUE CHANNEL
figure
stem(x,yBlue,'b')
title('Red Buoy: blue Channel')

%N = 5, D = 3 since RGB
%compute the model parameters (Mean and Covariance parameters
%of N D-dimension Gaussian) of the color distribution for each buoy

%CREATE  5 GAUSSIANS
Gmm = fitgmdist(double(Samples),6,'Regularize',.0001)%Samples has 3 columns, so it's 3 dimensionsal, 5 gaussians of 3 dimensions
GmmMean = Gmm.mu;
GmmVar = Gmm.Sigma;

%y = pdf(obj,X) returns a vector y of length n containing the values of the
%probability density function (pdf) for the gmdistribution object obj, 
%evaluated at the n-by-d data matrix X, where n is the number of observations
%and d is the dimension of the data. 
Test = [200 150 213; 105 116 200];
y = pdf(Gmm,Test);

%%
%YELLOW GMM LEARNING
imagefiles = dir('../Yellow/*.jpg')
cd('../Yellow')
nfiles = length(imagefiles);    % Number of files found
Samples = [];
for i=1:nfiles%for every cropped image
   currentfilename = imagefiles(i).name;
   frame = imread(currentfilename);
   
   %Get Channels
redChannel = frame(:, :, 1);
greenChannel = frame(:, :, 2);
blueChannel = frame(:, :, 3);

%Get arrays of every Channel value
[a b] = size(redChannel);%get dimensions of color channels
pixelR = [];%matrices for color channels. Basically want a straight array of values 0-255 for each pixel, for each color
pixelG = [];
pixelB = [];
for i = 1:a
    for j = 1:b
        pixelR = [pixelR redChannel(i,j)];%add value to color matrix
        pixelG = [pixelG greenChannel(i,j)];
        pixelB = [pixelB blueChannel(i,j)];
    end
end
pixelR = pixelR';%transpose to make 1 column
pixelG = pixelG';
pixelB = pixelB';
Samp = [pixelR pixelG pixelB];
Samples = [Samples ; Samp];
end

%HISTOGRAMS FOR EACH CHANNEL
figure
[yRed, x] = imhist(Samples(:,1));
stem(x, yRed,'r');%AFTER INSPECTION, JUST 1 GAUSSIAN FOR RED CHANNEL, Yellow BUOY
title('Yellow Buoy: red Channel')

[yGreen, x] = imhist(Samples(:,2));
figure
stem(x,yGreen,'g')%AFTER INSPECTION, 2 GAUSSIANS FOR GREEN CHANNEL, Yellow BUOY
title('Yellow Buoy: green Channel')

[yBlue, x] = imhist(Samples(:,3));%2 GUASSIANS FOR BLUE CHANNEL
figure
stem(x,yBlue,'b')
title('Yellow Buoy: blue Channel')

%N = 5, D = 3 since RGB
% compute the model parameters (Mean and Covariance parameters
% of N D-dimension Gaussian) of the color distribution for each buoy

%CREATE  4 GAUSSIANS
GmmY = fitgmdist(double(Samples),4,'Regularize',.0001)%Samples has 3 columns, so it's 3 dimensionsal, 5 gaussians of 3 dimensions
GmmMeanY = GmmY.mu;
GmmVarY = GmmY.Sigma;

%%
%GREEN BUOY
imagefiles = dir('../Green/*.jpg');
cd('../Green')
nfiles = length(imagefiles);    % Number of files found
Samples = [];
for i=1:nfiles%for every cropped image
   currentfilename = imagefiles(i).name;
   frame = imread(currentfilename);
   
   %Get Channels
redChannel = frame(:, :, 1);
greenChannel = frame(:, :, 2);
blueChannel = frame(:, :, 3);

%Get arrays of every Channel value
[a b] = size(redChannel);%get dimensions of color channels
pixelR = [];%matrices for color channels. Basically want a straight array of values 0-255 for each pixel, for each color
pixelG = [];
pixelB = [];
for i = 1:a
    for j = 1:b
        pixelR = [pixelR redChannel(i,j)];%add value to color matrix
        pixelG = [pixelG greenChannel(i,j)];
        pixelB = [pixelB blueChannel(i,j)];
    end
end
pixelR = pixelR';%transpose to make 1 column
pixelG = pixelG';
pixelB = pixelB';
Samp = [pixelR pixelG pixelB];
Samples = [Samples ; Samp];
end

%HISTOGRAMS FOR EACH CHANNEL
figure
[yRed, x] = imhist(Samples(:,1));
stem(x, yRed,'r');%AFTER INSPECTION, JUST 1 GAUSSIAN FOR RED CHANNEL, Yellow BUOY
title('Green Buoy: red Channel')

[yGreen, x] = imhist(Samples(:,2));
figure
stem(x,yGreen,'g')%AFTER INSPECTION, 2 GAUSSIANS FOR GREEN CHANNEL, Yellow BUOY
title('Green Buoy: green Channel')

[yBlue, x] = imhist(Samples(:,3));%2 GUASSIANS FOR BLUE CHANNEL
figure
stem(x,yBlue,'b')
title('Green Buoy: blue Channel')

%N = 5, D = 3 since RGB
% compute the model parameters (Mean and Covariance parameters
% of N D-dimension Gaussian) of the color distribution for each buoy

%CREATE  6 GAUSSIANS
GmmG = fitgmdist(double(Samples),7  ,'Regularize',.0001)%Samples has 3 columns, so it's 3 dimensionsal, 5 gaussians of 3 dimensions
GmmMeanG = GmmG.mu;
GmmVarG = GmmG.Sigma;

close all


