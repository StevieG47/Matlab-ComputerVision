
clear all
clc
%folder = 'C:\Users\Steve\Documents\Grad School\Perception\Project 1\Buoy\UPDATED\Images\TrainingSet\CroppedBuoys';
folder = '..\..\Images\TrainingSet\CroppedBuoys';%parent parent ....
filenames = dir(fullfile(folder, '*.jpg'));
filenamesGreen = dir(fullfile(folder, '*Green*.jpg'));%Contains all files with 'green' in it and is a jpg
filenamesRed = dir(fullfile(folder, '*Red*.jpg'));
filenamesYellow = dir(fullfile(folder, '*Yellow*.jpg'));

%RED BUOY COLOR DISTRIBUTION
RedData = [];
GreenData = [];
BlueData = [];
for i = 1:length(filenamesRed)%go through image that has 'Red' in it
full_name= fullfile(folder, filenamesRed(i).name);      %Read red buoy image sdaf
frame = imread(full_name);  


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

RedData = [RedData ; pixelR];
GreenData = [GreenData; pixelG];
BlueData = [BlueData ; pixelB];

Rdata = [RedData GreenData BlueData];

end
%Plot RGB value for every pixel from every cropped red buoy image
scatter3(RedData,GreenData,BlueData,5,'r');
title('Red Buoy Color Distribution')
xlabel('Red')
ylabel('Green')
zlabel('Blue')


f = getframe(gcf);
im = frame2im(f);
imwrite(im,'..\..\Output\Part0\R_Hist.jpg');

%%
%YELLOW BUOY COLOR DISTRIBUTION
RedData = [];
GreenData = [];
BlueData = [];
for i = 1:length(filenamesYellow)
full_name= fullfile(folder, filenamesYellow(i).name);      %Read yellow buoy image
frame = imread(full_name);  


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

RedData = [RedData ; pixelR];
GreenData = [GreenData; pixelG];
BlueData = [BlueData ; pixelB];

Ydata = [RedData GreenData BlueData];

end
%Plot RGB value for every pixel from every cropped Yellow buoy image
figure
scatter3(RedData,GreenData,BlueData,5,'y')
title('Yellow Buoy Color Distribution')
xlabel('Red')
ylabel('Green')
zlabel('Blue')
f = getframe(gcf);
im = frame2im(f);
imwrite(im,'..\..\Output\Part0\Y_Hist.jpg');

%%
%GREEN BUOY COLOR DISTRIBUTION
RedData = [];
GreenData = [];
BlueData = [];
for i = 1:length(filenamesYellow)
full_name= fullfile(folder, filenamesGreen(i).name);      %Read green buoy image
frame = imread(full_name);  


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

RedData = [RedData ; pixelR];
GreenData = [GreenData; pixelG];
BlueData = [BlueData ; pixelB];

Gdata = [RedData GreenData BlueData];

end
%Plot RGB value for every pixel from every cropped green buoy image
figure
scatter3(RedData,GreenData,BlueData,5,'g')
title('Green Buoy Color Distribution')
xlabel('Red')
ylabel('Green')
zlabel('Blue')
f = getframe(gcf);
im = frame2im(f);
imwrite(im,'..\..\Output\Part0\G_Hist.jpg');





        
