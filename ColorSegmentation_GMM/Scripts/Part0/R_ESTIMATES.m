
clear all
clc
imagefiles = dir('C:\\Users\\Steve\\Documents\\Grad School\\Perception\\Project 1\\Buoy\\UPDATED\\Images\\TrainingSet\\CroppedBuoys\\Red\\*.jpg');      
cd('C:\\Users\\Steve\\Documents\\Grad School\\Perception\\Project 1\\Buoy\\UPDATED\\Images\\TrainingSet\\CroppedBuoys\\Red')
% FF = dir('..\\..\\Scripts\\Part0')

    
nfiles = length(imagefiles);    % Number of files found
C = cell(nfiles,1);
RedSamples = [];
for i=1:nfiles
   currentfilename = imagefiles(i).name;
   frame = imread(currentfilename);
   
redChannel = frame(:, :, 1);
greenChannel = frame(:, :, 2);
blueChannel = frame(:, :, 3);

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

RedSamples = [RedSamples ; pixelR];
end

%Do Gaussian 
%Make gaussian from randomly selected sample from values in the redChannel

%Take random samples add to matrix called Data
Data = [];%Array holding all randomly selected samples
[sz s] = size(RedSamples);
for i = 1:100000
a = randi([1 sz]);%pick random element number
chosen = RedSamples(a);%take element from pixelR
Data = [Data chosen];%Add randomly chosen element to Data
end
Data = Data';

%Random distribution represented as gaussian with mean, stnd dev
%This is the data for the red buoy
meanR = mean(Data)%mean of Data
covR = std(double(Data))%variance of Data



   
   
   
   
   
   
   
  