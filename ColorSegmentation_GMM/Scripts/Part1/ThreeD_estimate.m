clear all
clc
imagefiles = dir('C:\\Users\\Steve\\Documents\\Grad School\\Perception\\Project 1\\Buoy\\UPDATED\\Images\\TrainingSet\\CroppedBuoys\\Red\\*.jpg');      
cd('C:\\Users\\Steve\\Documents\\Grad School\\Perception\\Project 1\\Buoy\\UPDATED\\Images\\TrainingSet\\CroppedBuoys\\Red')
nfiles = length(imagefiles);    % Number of files found
C = cell(nfiles,1);
Samples = [];

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

Pix = [pixelR pixelG pixelB];
Samples = [Samples; Pix];
end

%use mvn ( [input RGB], [1x3 mean], [covariance] to get probability density
%function, this isnt probability just function. can use stand dev, sqrt
%root of top left of covar to know stand between 2 stnd dev is 68% Find teh
%covar matrix by doing stnd dev of each channgel to get diagonal; Do
%cov(R,G) and use top right and bottom left terms in the 3x3 corresponding
%spots. to get covariance matrix, make sure eigenvalues of covariance are
%not negative. Need way to find probability once probability density
%funcion is found. 
%Need like 30 frames to train for 1d, at least 60 frames to train for 3d.

%Do Gaussian 
%Make gaussian from randomly selected sample from [RGB} values of red buoy

Data = [];%Array holding all randomly selected samples
[sz s] = size(Samples);
for i = 1:100000
a = randi([1 sz]);%pick random element number
chosen = Samples(a,:);%take element from pixelR
Data = [Data ;chosen];%Add randomly chosen element to Data
end
R = Data(:,1);
G = Data(:,2);
B = Data(:,3);
R = double(R);
G = double(G);
B = double(B);
%Mean of R,G, B
meanR = mean(Data(:,1));
meanG = mean(Data(:,2));
meanB = mean(Data(:,3));
mean = [meanR meanG meanB]

%Stnd dev of R,G,B
stdR = std(double(Data(:,1)));
stdG = std(double(Data(:,2)));
stdB = std(double(Data(:,3)));

%Covariance 
%Red green
Crg = cov(R,G);
covarRG = Crg(1,2);
covarGR = Crg(2,1);
%red blue
Crb = cov(R,B);
covarRB = Crb(1,2);
covarBR = Crb(2,1);
%green blue
Cgb = cov(G,B);
covarGB = Cgb(1,2);
covarBG = Cgb(2,1);

covar = [stdR^2  covarRG  covarRB;
         covarGR stdG^2   covarGB;
         covarBR covarBG  stdB^2;]
       
   
% %Gaussian for individual R,G,B
% Pr = normpdf(double(redChannel),meanR,stdR);
% Pg = normpdf(double(greenChannel),meanG,stdG);
% Pb = normpdf(double(blueChannel),meanB,stdB);




%View gaussian of channel
x = [0:10:300];
norm = normpdf(x,meanR,stdR);
plot(x,norm,'r','LineWidth',3)
hold on
norm1 = normpdf(x,meanG,stdG);
plot(x,norm1,'g','LineWidth',3)

norm2 = normpdf(x,meanB,stdB);
plot(x,norm2,'b','LineWidth',3)


