clear all
clc
imagefiles = dir('C:\\Users\\Steve\\Documents\\Grad School\\Perception\\Project 1\\Buoy\\UPDATED\\Images\\TrainingSet\\CroppedBuoys\\Green\\*.jpg');      
cd('C:\\Users\\Steve\\Documents\\Grad School\\Perception\\Project 1\\Buoy\\UPDATED\\Images\\TrainingSet\\CroppedBuoys\\Green')
nfiles = length(imagefiles);    % Number of files found
C = cell(nfiles,1);
GreenSamples = [];
for i=1:nfiles
   currentfilename = imagefiles(i).name;
   frame = imread(currentfilename);
   
redChannel = frame(:, :, 1);
greenChannel = frame(:, :, 2);
blueChannel = frame(:, :, 3);
yellowChannel = (double(redChannel)+double(greenChannel))/2;

%greenChannel = (double(greenChannel)+double(blueChannel))/2;

[a b] = size(greenChannel);%get dimensions of color channels
pixelR = [];%matrices for color channels. Basically want a straight array of values 0-255 for each pixel, for each color
pixelG = [];
pixelB = [];
pixelY = [];
for i = 1:a
    for j = 1:b
        
        pixelG = [pixelG greenChannel(i,j)];
    end
end
pixelR = pixelR';%transpose to make 1 column
pixelG = pixelG';
pixelB = pixelB';
pixelY = pixelY';

GreenSamples = [GreenSamples ; pixelG];
end

%Do Gaussian 
%Make gaussian from randomly selected sample from values in the redChannel

%Take random samples add to matrix called Data
Data = [];%Array holding all randomly selected samples
[sz s] = size(GreenSamples);
for i = 1:300000
a = randi([1 sz]);%pick random element number
chosen = GreenSamples(a);%take element from pixelR
Data = [Data chosen];%Add randomly chosen element to Data
end
Data = Data';

%Random distribution represented as gaussian with mean, stnd dev
u = mean(Data)%mean of Data
s = std(double(Data))%variance of Data