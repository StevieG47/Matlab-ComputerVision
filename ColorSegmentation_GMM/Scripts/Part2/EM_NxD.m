clear all 
clc
%takes number of gaussians N, data and plot path as inputs and outputs
%the the array N × D mean and N × D × D covariance model parameters. The
%function should plot the computed Gaussians and save
%it to the relative path mentioned in plot path 

N = 3;%number of gaussians
D = 1;%dimensions
Data = [];%Array of data to be taken as input

%Lets make our own data for now
MeanList = zeros(N,D);
VarList = zeros(N,D);
for g = 1:N
    for h = 1:D
    randMean = randi(20);
    MeanList(g,h) = randMean;% = [MeanList  randMean];
    
    randV = randi(10);
    VarList(g,h) = randV;% = [VarList randV];
    end

end%Created N means and N variances

Data = [];
Data2 = [];
for p = 1:length(MeanList)
    for q = 1:D
        samp = mvnrnd(MeanList(p,q),VarList(p,q),100);%create samples for each mean/std combo
        Data = [Data samp];%Create 1 long array of all samples
    end
    Data2 = [Data2 ; Data];
    Data = [];
     
end%Add samples from every mean/var to a long Data list

% x  = (-10:1:30);
% for b = 1:length(MeanList)
%     pd = normpdf(x,MeanList(b),VarList(b)^.5);
%     plot(x,pd)
%     hold on
% end
%figure

Gmm = fitgmdist(Data2,N);%Fit N gaussians to all samples taken
MeanList;%Display original mean data 
FoundMean = Gmm.mu%N means of D dimensionality
FoundCov = Gmm.Sigma%there are N  DxD matrices representing covariance matrices for each gaussian






