clear all
clc

% two finger scroll disable
% !synclient HorizEdgeScroll=0 HorizTwoFingerScroll=0 

%saturation can bump up individual colors
%brightness all 3 colors together
vid = VideoReader('../Data/videos/Tag2.mp4');%Read video, Tag 0 vid used first 4 secodns, Tag1 used last 4 seconds
%Make new vid
NewVid = VideoWriter('../Output/Lena_output_Tag2');%Writin the new video
open(NewVid);%open the new video
Lena = imread('../Data/Lena.png');
messedUpH = 0;%how many guesses were made
runs = 0;%how many run throughs
good_corners = 0;%define so if we fail to have corners theres no error

for nnn = 1:round(vid.Duration*vid.FrameRate)-1%frames to iterate through

frame = read(vid,nnn);%Current frame

G = fspecial('gaussian', [5,5],2);
I = imfilter(frame,G,'same');%gaussian filter
I = rgb2gray(frame);%change to gray scale

edgeDet = edge(I,'Roberts');%edge detection

%Dilation adds pixels to the boundaries of objects in an image, while erosion removes pixels on object boundaries.
%dilation changes to 1 if 1 around, erosion changes to 0 if 0 around
%strel is structuring element
se1 = strel('disk', 5);
se2 = strel('disk', 4);
edgeDet = imdilate(edgeDet,se1);%dilates the image, se1 is structure element argument
edgeDet = imerode(edgeDet,se2);%erodes the image, same deal with se2
edgeDet = bwareaopen(edgeDet,20);%Get rid of small noise

%imshow(frame)



%hold on
no = 0;%testing var
B = bwboundaries(edgeDet, 8, 'noholes'); % Boundary 
CurMaxA = 0;%For area filter when finding corners
for i=1:length(B)
    Bc = B(i);
    for j=1:length(Bc)
        Bc = Bc{j};
        pol = dpsimplify(Bc,10); %simplifying the polygons/lines so we can find things with four corners
        pol_size = size(pol);
        
        if(pol_size(1) == 5) %Def want 4 corners
            for a=1:pol_size(1)-1
                for b=1:2
                  cornersFound(a,b) = pol(a,b); %4corners
                                    
                end
            end
            
           %Only use set of corners that have points more than 36 pixels
           %away, this gets rid of all tiny bs corner finds
           %Maybe use polyarea instead
            areaLimit = 36;
           if( ((cornersFound(2,2) - cornersFound(1,2))^2 + (cornersFound(2,1)-cornersFound(1,1))^2)^.5 < areaLimit || ((cornersFound(4,2) - cornersFound(1,2))^2 + (cornersFound(4,1)-cornersFound(1,1))^2)^.5 < areaLimit ...
                   ||((cornersFound(3,2) - cornersFound(1,2))^2 + (cornersFound(3,1)-cornersFound(1,1))^2)^.5 < areaLimit || ((cornersFound(3,2) - cornersFound(2,2))^2 + (cornersFound(3,1)-cornersFound(2,1))^2)^.5 < areaLimit ...
                   || polyarea(cornersFound(:,1),cornersFound(:,2))<1000)
               no = no+1;
            %This if checks distances between corners and area, if they arent big enough we get rid of them   
           else
               if(polyarea(cornersFound(:,2),cornersFound(:,1)) > CurMaxA)%If the area between corners is bigger than current biggest area
               CurMaxA = polyarea(cornersFound(:,2),cornersFound(:,1));%Set this new area to be the newest biggest area
                good_corners = cornersFound;
               end
              
            
           end

           
            
       end
    end
   
end
 %plot(good_corners(:,2),good_corners(:,1),'ro')

if good_corners == 0
 C = cornersFound;
else
C = good_corners;%locations of corners, (y,x)
end
C = [C(:,2),C(:,1)];%make locations x,y
xc = C(:,1)';
yc = C(:,2)';
%new image location n pixels long and wide.
n = 200;%reference tag this size
Q(1,:) = [1, 1];
Q(2,:) = [n, 1];
Q(3,:) = [n, n];
Q(4,:) = [1, n];

%USed with peter K's homography 2d.m
%qx = Q(:,1)';
%qy = Q(:,2)';
%x1 = [xc;yc;1 1 1 1];
%x2 = [qx;qy;1 1 1 1];
%H1 = homography2d(x1, x2);
[l,w,a] = size(frame);

%To be used with imtransform
%T = maketform('projective',[Q(:,1) Q(:,2)],[C(:,1) C(:,2)]);
%H = T.tdata.T;

%to be used with imwarp
%fitgeotrans(movingPoints,fixedPoints,'projective');
H = fitgeotrans([C(:,1) C(:,2)],[Q(:,1) Q(:,2)],'projective');%geometric transformation, x1,y1,x2,y2. 
R=imref2d(size(frame),[1 size(frame,2)],[1 size(frame,1)]);
frame2 = imwarp(frame,H,'OUTPUTVIEW',R);%create warped image using the H found
Tag = imcrop(frame2,[1 1 n-1 n-1]);%crop to only include max pixel size of what we set before, in Q
%figure(2)
%imshow(Tag);

%identifiyng a tag. Make 8x8 grid and check if black or white, 0 or 1

step = n/8;
G1 = int16(step/2);
G2 = int16(G1 + step);
G3 = int16(G2 + step);
G4 = int16(G3 + step);
G5 = int16(G4 + step);
G6 = int16(G5 + step);
G7 = int16(G6 + step);
G8 = int16(G7 + step);
G = [G1,G2,G3,G4,G5,G6,G7,G8];%Each G is center of box when divided up into an 8x8 grid
grid = zeros(numel(G),numel(G));%0 means black, 1 means white, will change white boxes to 1 next

for k=1:numel(G)%take center box coord for each 8x8 box and see if it's white or black
    for j=1:numel(G)
        if Tag(G(k),G(j),1)> 210 && Tag(G(k),G(j),2)>210 && Tag(G(k),G(j),3)>210
           grid(k,j) = 1;%if it's white change corresponsing poisition in grid to 1
        end
    end
end


%figure(3)
T = Tag;%Tag is homogrophized image, T will be a nice looking black or white image used to rotate and stuff
%%CREATE T, THE TAG IMAGE ALL NICE
for p=1:8%foing thorugh grid
    for o = 1:8%going through grid
        if grid(p,o) == 1%matching grid to this new T image, so if white on grid, or 1, write correponding T grid box to white
            for AA = (p-1)*step:p*step%go through pixels of specified box and change color
                for BB = (o-1)*step:o*step
                    if(AA == 0)%%cant have a 0 cause the first pixel is 1
                        AA = 1;
                    end
                    if (BB == 0)
                        BB = 1;
                    end
                    
                    T(AA,BB,1) = 255;%So set the pixel to color white, RGB all equal 255, if the grid has a 1 value
                    T(AA,BB,2) = 255;
                    T(AA,BB,3) = 255;
                    %We know there is a padding of black on the outside so
                    %lets ensure that
                     if ((p==1 || p ==2 || p == 7 || p==8) || (o == 1 || o ==2 || o == 7 || o == 8))
                      T(AA,BB,1) = 0;%So set the pixel to colorblack cause the padding is def black
                      T(AA,BB,2) = 0;
                      T(AA,BB,3) = 0;
                      
                     end
                    end
                end
        end
            
    
        if grid(p,o) == 0%Same thing except with color white now
            for AA = (p-1)*step:p*step
                for BB = (o-1)*step:o*step
                    if(AA == 0)
                        AA = 1;
                    end
                    if (BB == 0)
                        BB = 1;
                    end
                    T(AA,BB,1) = 0;
                    T(AA,BB,2) = 0;
                    T(AA,BB,3) = 0;
                                       
                end
            end
        end
        
    end
end
%figure
%imshow(T)

%rotate so bottom right of 4x4 inner white pattern has bottom right
countR = 0;
for r = 1:3%max of 3 rotations
    if grid(6,6) == 0 %if grid spot is 0, or black
        T = imrotate(T,90);%rotate counter clockwise
        countR = countR +1;
        %GET THE NEW GRID
        for k=1:8%take center box coord for each 8x8 box and see if it's white or black
             for j=1:8
                 if T(G(k),G(j),1)> 210 && T(G(k),G(j),2)>210 && T(G(k),G(j),3)>210
                 grid(k,j) = 1;%if it's white change corresponsing poisition in grid to 1
                end
             end
        end
    end
end

%IF we detct multiple white corners in the 4x4 grid, the homography fucked
%up, we dont know which way is correct. The best guess (I can come up with)
%is to assume it has the same orientation as the previous frame
if (runs>1)
    if ( T(G(6),G(6),1) > 200 && ( T(G(3),G(3),1) > 200 || T(G(3),G(6),1) > 200 || T(G(6),G(3),1) > 200) )
        countR = countRprev;
        messedUpH = messedUpH + 1;
    end
end


%figure
%imshow(T)
% hold on
% scatter(1,1,200,'r','filled')
% scatter(1,n,200,'y','filled')
% scatter(n,n,200,'b','filled')
% scatter(n,1,200,'g','filled')

%PLOT CIRCLES IN THE CORRECT POSITION DEPENDING ON HOW MANY ROTATIONS THERE
%WERE
%Plot them in correct positions first then rotate to match what was rotated
%before. THIS PLOTS ON HOMOGRAPHIZED TAG
%figure 
%imshow(Tag)
%hold on
% if(countR == 0)
% scatter(1,1,200,'r','filled')
% scatter(1,n,200,'y','filled')
% scatter(n,n,200,'b','filled')
% scatter(n,1,200,'g','filled')
% end
% if(countR == 1)
% scatter(1,1,200,'y','filled')
% scatter(1,n,200,'b','filled')
% scatter(n,n,200,'g','filled')
% scatter(n,1,200,'r','filled')
% end
% if(countR == 2)
% scatter(1,1,200,'b','filled')
% scatter(1,n,200,'g','filled')
% scatter(n,n,200,'r','filled')
% scatter(n,1,200,'y','filled')
% end
% if(countR == 3)
% scatter(1,1,200,'g','filled')
% scatter(1,n,200,'r','filled')
% scatter(n,n,200,'y','filled')
% scatter(n,1,200,'b','filled')
% end

%PLOT ON ACTUAL FRAME
%C has x and y variables
%figure
%imshow(frame)
%hold on
% if(countR == 0) 
% scatter(C(1,1),C(1,2),100,'r','filled')
% scatter(C(2,1),C(2,2),100,'g','filled')
% scatter(C(3,1),C(3,2),100,'b','filled')
% scatter(C(4,1),C(4,2),100,'y','filled')
% end
% 
% if(countR == 1)
% scatter(C(1,1),C(1,2),100,'y','filled')
% scatter(C(2,1),C(2,2),100,'r','filled')
% scatter(C(3,1),C(3,2),100,'g','filled')
% scatter(C(4,1),C(4,2),100,'b','filled')
% end
% 
% if(countR == 2)
% scatter(C(1,1),C(1,2),100,'b','filled')
% scatter(C(2,1),C(2,2),100,'y','filled')
% scatter(C(3,1),C(3,2),100,'r','filled')
% scatter(C(4,1),C(4,2),100,'g','filled')
% end
% 
% if(countR == 3)
% scatter(C(1,1),C(1,2),100,'g','filled')
% scatter(C(2,1),C(2,2),100,'b','filled')
% scatter(C(3,1),C(3,2),100,'y','filled')
% scatter(C(4,1),C(4,2),100,'r','filled')
% end


%figure1 = figure;
%saveas(figure,'Test.jpg')


%figure
%imshow(frame)
%hold on

%LENA TIME
%fitgeotrans(movingPoints,fixedPoints,'projective');
%Lena is 512 by 512
%countR = 0;%testing
%Orient Lena to be same as Tag before rotation
if (countR == 0)
    L = [1 1;  512 1; 512 512; 1 512;];
end
if (countR == 1)
    L = [1 512; 1 1; 512 1; 512 512];
end
if (countR == 2)
    L = [512 512; 1 512; 1 1; 512 1;];
end
if (countR == 3)
    L = [512 1; 512 512; 1 512; 1 1;];
end
HH = fitgeotrans([L(:,1) L(:,2)],[C(:,1) C(:,2)],'projective');%geometric transformation, x1,y1,x2,y2. 
RR=imref2d(size(frame),[1 size(frame,2)],[1 size(frame,1)]);%%reference is same size as frame
Lframe = imwarp(Lena,HH,'OUTPUTVIEW',RR);%create warped image using the H found
%imshow(Lframe)

[M,N] = size(Lframe);
%Lena homographized image is same size as frame, but everything is black
%but lena, so go thourhg and if pixel is black change it to color of scene,
%or color in frame
for i=1:1080
   for j=1:1920
      if(Lframe(i,j,1) == 0 && Lframe(i,j,2) == 0 && Lframe(i,j,3) == 0)   % if black change it to actual scene
         Lframe(i,j,1) = frame(i,j,1);
         Lframe(i,j,2) = frame(i,j,2);
         Lframe(i,j,3) = frame(i,j,3);
      end
   end
end

%title(nnn)
%imshow(Lframe) %Final Image of lena on scene


writeVideo(NewVid,Lframe) %Add frame to video
countRprev = countR;
runs = runs+1
end

close(NewVid);
%set (gcf, 'WindowButtonMotionFcn', @mouseMove);
    
