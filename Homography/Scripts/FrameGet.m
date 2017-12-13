clear all
clc

%saturation can bump up individual colors
%brightness all 3 colors together
vid = VideoReader('Tag2.mp4');%Read video, Tag 0 vid used first 4 secodns, Tag1 used last 4 seconds
%Make new vid
%NewVid = VideoWriter('CUBETEST.mp4');
%open(NewVid);
Lena = imread('Lena.png');
ref = imread('ref_marker.png');
messedUpH = 0;%how many guesses were made
runs = 0;%how many run throughs
good_corners = 0;%define so if we fail to have corners theres no erro

%K Matrix
 K = [629.302552 0 330.766408; 
     0 635.529018 251.004731; 0 0 1;];  %K = [fx 0 0; s fy 0; cx cy 0;]
 
 K = [1406.08415449821        0           0;
    2.20679787308599   1417.99930662800   0;
    1014.13643417416, 566.347754321696    1]';


for nnn = 270:1:390%Run through frames of video
frame = read(vid,nnn);
% G = fspecial('gaussian', [5,5],2);
% I = imfilter(frame,G,'same');%gaussian filter
% I = rgb2gray(frame);%change to gray scale
% 
% edgeDet = edge(I,'Roberts');%edge detection
% 
% %Dilation adds pixels to the boundaries of objects in an image, while erosion removes pixels on object boundaries.
% %dilation changes to 1 if 1 around, erosion changes to 0 if 0 around
% %strel is structuring element
% se1 = strel('disk', 5);
% se2 = strel('disk', 4);
% edgeDet = imdilate(edgeDet,se1);%dilates the image, se1 is structure element argument
% edgeDet = imerode(edgeDet,se2);%erodes the image, same deal with se2
% edgeDet = bwareaopen(edgeDet,20);
% 
% 
% %hold on
% no = 0;%testing var
% B = bwboundaries(edgeDet, 8, 'noholes'); % Boundary 
% CurMaxA = 0;%For area filter when finding corners
% for i=1:length(B)
%     Bc = B(i);
%     for j=1:length(Bc)
%         Bc = Bc{j};
%         pol = dpsimplify(Bc,10); %simplifying the polygons/lines so we can find things with four corners
%         pol_size = size(pol);
%         
%         if(pol_size(1) == 5) %Def want 4 corners
%             for a=1:pol_size(1)-1
%                 for b=1:2            
%                     cornersFound(a,b) = pol(a,b); %4corners
%                    
%                     
%                 end
%             end
%             
%            %Only use set of corners that have points more than 36 pixels
%            %away, this gets rid of all tiny bs corner finds
%            %Maybe use polyarea instead
%              areaLimit = 25;
%            if( ((cornersFound(2,2) - cornersFound(1,2))^2 + (cornersFound(2,1)-cornersFound(1,1))^2)^.5 < areaLimit || ((cornersFound(4,2) - cornersFound(1,2))^2 + (cornersFound(4,1)-cornersFound(1,1))^2)^.5 < areaLimit ...
%                    ||((cornersFound(3,2) - cornersFound(1,2))^2 + (cornersFound(3,1)-cornersFound(1,1))^2)^.5 < areaLimit || ((cornersFound(3,2) - cornersFound(2,2))^2 + (cornersFound(3,1)-cornersFound(2,1))^2)^.5 < areaLimit ...
%                    || polyarea(cornersFound(:,1),cornersFound(:,2))<500)
%                no = no+1;
%             %This if checks distances between corners and area, if they arent big enough we get rid of them   
%            else
%                if(polyarea(cornersFound(:,2),cornersFound(:,1)) > CurMaxA)%If the area between corners is bigger than current biggest area
%                CurMaxA = polyarea(cornersFound(:,2),cornersFound(:,1));%Set this new area to be the newest biggest area
%                 good_corners = cornersFound;
%                  %plot(good_corners(:,2),good_corners(:,1),'ro')
%                end
%               
%             
%            end
%        
% 
%            
%             
%        end
%     end
%    
% end
% 
% if good_corners == 0
%     C = cornersFound;
% else
% C = good_corners;%locations of corners, (y,x)
% end
% C = [C(:,2),C(:,1)];%make locations x,y
% xc = C(:,1)';
% yc = C(:,2)';
% %new image location n pixels long and wide.
% n = 200;
% Q(1,:) = [1, 1];
% Q(2,:) = [n, 1];
% Q(3,:) = [n, n];
% Q(4,:) = [1, n];
% 
% %Using ref marker instead of Q
% 
% %USed with peter K's homography 2d.m
% qx = Q(:,1)';
% qy = Q(:,2)';
% x1 = [xc;yc;1 1 1 1];
% x2 = [qx;qy;1 1 1 1];
% %Try with unit square
% UnitQ(1,:) = [0, 0];
% UnitQ(2,:) = [1, 0];
% UnitQ(3,:) = [1, 1];
% UnitQ(4,:) = [0, 1];
% qx = UnitQ(:,1)';
% qy = UnitQ(:,2)';
% x2 = [qx;qy;1 1 1 1];
% 
% H1 = homography2d(x2, x1);
% R=[840 933 802 968];
% [newim, newT] = imTrans(frame, H1);
% %figure
% %imshow(newim)
% 
% %To be used with imtransform
% %T = maketform('projective',[Q(:,1) Q(:,2)],[C(:,1) C(:,2)]);
% %H = T.tdata.T;
% 
% %to be used with imwarp
% %fitgeotrans(movingPoints,fixedPoints,'projective');
% H = fitgeotrans([C(:,1) C(:,2)],[Q(:,1) Q(:,2)],'projective');%geometric transformation, x1,y1,x2,y2. 
% R=imref2d(size(frame),[1 size(frame,2)],[1 size(frame,1)]);
% frame2 = imwarp(frame,H,'OUTPUTVIEW',R);%create warped image using the H found
% Tag = imcrop(frame2,[1 1 n-1 n-1]);%crop to only include max pixel size of what we set before, in Q
% %figure(2)
% %imshow(Tag);
% 
% % %TRY NEW HOMOGRAPHY FOLLOWING PDF
% % world = UnitQ;
% % cam = C;
% % %x and y world points
% % x1w = world(1,1);
% % y1w = world(1,2);
% % x2w = world(2,1);
% % y2w = world(2,2);
% % x3w = world(3,1);
% % y3w = world(3,2);
% % x4w = world(4,1);
% % y4w = world(4,2);
% % %x and y cam points
% % x1c = cam(1,1);
% % y1c = cam(1,2);
% % x2c = cam(2,1);
% % y2c = cam(2,2); 
% % x3c = cam(3,1);
% % y3c = cam(3,2);
% % x4c = cam(4,1);
% % y4c = cam(4,2);
% % %Find A matrix from equation (7)
% % A1 = [x1w y1w 1 0 0 0 -x1c*x1w -x1c*y1w -x1c;
% %       0 0 0 x1w y1w 1 -y1c*x1w -y1c*y1w -y1c;];
% % A2 = [x2w y2w 1 0 0 0 -x2c*x2w -x2c*y2w -x2c;
% %       0 0 0 x2w y2w 1 -y2c*x2w -y2c*y2w -y2c;];
% % A3 = [x3w y3w 1 0 0 0 -x3c*x3w -x3c*y3w -x3c;
% %       0 0 0 x3w y3w 1 -y3c*x3w -y3c*y3w -y3c;];
% % A4 = [x4w y4w 1 0 0 0 -x4c*x4w -x4c*y4w -x4c;
% %       0 0 0 x4w y4w 1 -y4c*x4w -y4c*y4w -y4c;];
% % %Final A matrix
% % A = [A1;A2;A3;A4];
% % %Do SVD of A
% % [U, S, VT] = svd(A);
% % V = VT';%so that we can say V = [values]^T
% % h = [V(1,9) V(2,9) V(3,9) V(4,9) V(5,9) V(6,9) V(7,9) V(8,9) V(9,9)]';
% % h = h/V(9,9);
% % hhh = reshape(h,3,3)';
% % [newim2, newT] = imTrans(frame, hhh);
% 
% 
% % identifiyng a tag. Make 8x8 grid and check if black or white, 0 or 1
% 
% step = n/8;
% G1 = int16(step/2);
% G2 = int16(G1 + step);
% G3 = int16(G2 + step);
% G4 = int16(G3 + step);
% G5 = int16(G4 + step);
% G6 = int16(G5 + step);
% G7 = int16(G6 + step);
% G8 = int16(G7 + step);
% G = [G1,G2,G3,G4,G5,G6,G7,G8];%Each G is center of box when divided up into an 8x8 grid
% grid = zeros(numel(G),numel(G));%0 means black, 1 means white, will change white boxes to 1 next
% 
% for k=1:numel(G)%take center box coord for each 8x8 box and see if it's white or black
%     for j=1:numel(G)
%         if Tag(G(k),G(j),1)> 210 && Tag(G(k),G(j),2)>210 && Tag(G(k),G(j),3)>210
%            grid(k,j) = 1;%if it's white change corresponsing poisition in grid to 1
%         end
%     end
% end
% 
% 
% %figure(3)
% T = Tag;%Tag is homogrophized image, T will be a nice looking black or white image used to rotate and stuff
% %%CREATE T, THE TAG IMAGE ALL NICE
% for p=1:8%foing thorugh grid
%     for o = 1:8%going through grid
%         if grid(p,o) == 1%matching grid to this new T image, so if white on grid, or 1, write correponding T grid box to white
%             for AA = (p-1)*step:p*step%go through pixels of specified box and change color
%                 for BB = (o-1)*step:o*step
%                     if(AA == 0)%%cant have a 0 cause the first pixel is 1
%                         AA = 1;
%                     end
%                     if (BB == 0)
%                         BB = 1;
%                     end
%                     
%                     T(AA,BB,1) = 255;%So set the pixel to color white, RGB all equal 255, if the grid has a 1 value
%                     T(AA,BB,2) = 255;
%                     T(AA,BB,3) = 255;
%                     %We know there is a padding of black on the outside so
%                     %lets ensure that
%                      if ((p==1 || p ==2 || p == 7 || p==8) || (o == 1 || o ==2 || o == 7 || o == 8))
%                       T(AA,BB,1) = 0;%So set the pixel to colorblack cause the padding is def black
%                       T(AA,BB,2) = 0;
%                       T(AA,BB,3) = 0;
%                       
%                      end
%                     end
%                 end
%         end
%             
%     
%         if grid(p,o) == 0%Same thing except with color white now
%             for AA = (p-1)*step:p*step
%                 for BB = (o-1)*step:o*step
%                     if(AA == 0)
%                         AA = 1;
%                     end
%                     if (BB == 0)
%                         BB = 1;
%                     end
%                     T(AA,BB,1) = 0;
%                     T(AA,BB,2) = 0;
%                     T(AA,BB,3) = 0;
%                                        
%                 end
%             end
%         end
%         
%     end
% end
% %figure
% %imshow(T)
% 
% %rotate so bottom right of 4x4 inner white pattern has bottom right
% countR = 0;
% for r = 1:3%max of 3 rotations
%     if grid(6,6) == 0 %if grid spot is 0, or black
%         T = imrotate(T,90);%rotate counter clockwise
%         countR = countR +1;
%         %GET THE NEW GRID
%         for k=1:8%take center box coord for each 8x8 box and see if it's white or black
%              for j=1:8
%                  if T(G(k),G(j),1)> 210 && T(G(k),G(j),2)>210 && T(G(k),G(j),3)>210
%                  grid(k,j) = 1;%if it's white change corresponsing poisition in grid to 1
%                 end
%              end
%         end
%     end
% end
% 
% %IF we detct multiple white corners in the 4x4 grid, the homography fucked
% %up, we dont know which way is correct. The best guess (I can come up with)
% %is to assume it has the same orientation as the previous frame
% if (runs>1)
%     if ( T(G(6),G(6),1) > 200 && ( T(G(3),G(3),1) > 200 || T(G(3),G(6),1) > 200 || T(G(6),G(3),1) > 200) )
%         countR = countRprev;
%         messedUpH = messedUpH + 1
%     end
% end
% 
% 
% %figure
% %imshow(T)
% % hold on
% % scatter(1,1,200,'r','filled')
% % scatter(1,n,200,'y','filled')
% % scatter(n,n,200,'b','filled')
% % scatter(n,1,200,'g','filled')
% 
% %PLOT CIRCLES IN THE CORRECT POSITION DEPENDING ON HOW MANY ROTATIONS THERE
% %WERE
% %Plot them in correct positions first then rotate to match what was rotated
% %before. THIS PLOTS ON HOMOGRAPHIZED TAG
% %figure 
% %imshow(Tag)
% %hold on
% % if(countR == 0)
% % scatter(1,1,200,'r','filled')
% % scatter(1,n,200,'y','filled')
% % scatter(n,n,200,'b','filled')
% % scatter(n,1,200,'g','filled')
% % end
% % if(countR == 1)
% % scatter(1,1,200,'y','filled')
% % scatter(1,n,200,'b','filled')
% % scatter(n,n,200,'g','filled')
% % scatter(n,1,200,'r','filled')
% % end
% % if(countR == 2)
% % scatter(1,1,200,'b','filled')
% % scatter(1,n,200,'g','filled')
% % scatter(n,n,200,'r','filled')
% % scatter(n,1,200,'y','filled')
% % end
% % if(countR == 3)
% % scatter(1,1,200,'g','filled')
% % scatter(1,n,200,'r','filled')
% % scatter(n,n,200,'y','filled')
% % scatter(n,1,200,'b','filled')
% % end
% figure
% imshow(frame)
% 
% %PLOT SOME CUBE ACTION
% 
% % side1 = ((C(2,2) - C(1,2))^2 + (C(2,1)-C(1,1))^2)^.5;
% % 
% % %vertical lines coming up from the tag
% % Line1Vertx = [C(1,1) C(1,1)];
% % Line1Verty = [C(1,2) C(1,2)-side1];
% % Line2Vertx = [C(2,1) C(2,1)];
% % Line2Verty = [C(2,2) C(2,2)-side1];
% % Line3Vertx = [C(3,1) C(3,1)];
% % Line3Verty = [C(3,2) C(3,2)-side1];
% % Line4Vertx = [C(4,1) C(4,1)];
% % Line4Verty = [C(4,2) C(4,2)-side1];
% % 
% % %Upper plane lines of the cube
% % U12x = [C(1,1) C(2,1)];
% % U12y = [C(1,2)-side1 C(2,2)-side1];
% % U23x = [C(2,1) C(3,1)];
% % U23y = [C(2,2)-side1 C(3,2)-side1];
% % U34x = [C(3,1) C(4,1)];
% % U34y = [C(3,2)-side1 C(4,2)-side1];
% % U41x = [C(4,1) C(1,1)];
% % U41y = [C(4,2)-side1 C(1,2)-side1];
% % 
% % 
% 
% % plot(Line1Vertx,Line1Verty,'k','LineWidth',lineWidth);
% % plot(Line2Vertx,Line2Verty,'k','LineWidth',lineWidth);
% % plot(Line3Vertx,Line3Verty,'k','LineWidth',lineWidth);
% % plot(Line4Vertx,Line4Verty,'k','LineWidth',lineWidth);
% % plot(U12x,U12y,'k','LineWidth',lineWidth);
% % plot(U23x,U23y,'k','LineWidth',lineWidth);
% % plot(U34x,U34y,'k','LineWidth',lineWidth);
% % plot(U41x,U41y,'k','LineWidth',lineWidth);
% 
% 
% %PLOT THE DOTS, CORNER OF CUBE
% %C has x and y variables
% %PLOTTING BASE CORNERS OF CUBE, just corners of the tag
% hold on
% circSize = 60;
% if(countR == 0) 
% scatter(C(1,1),C(1,2),circSize,'r','filled')
% scatter(C(2,1),C(2,2),circSize,'g','filled')
% scatter(C(3,1),C(3,2),circSize,'b','filled')
% scatter(C(4,1),C(4,2),circSize,'y','filled')
% end
% 
% if(countR == 1)
% scatter(C(1,1),C(1,2),circSize,'y','filled')
% scatter(C(2,1),C(2,2),circSize,'r','filled')
% scatter(C(3,1),C(3,2),circSize,'g','filled')
% scatter(C(4,1),C(4,2),circSize,'b','filled')
% end
% 
% if(countR == 2)
% scatter(C(1,1),C(1,2),circSize,'b','filled')
% scatter(C(2,1),C(2,2),circSize,'y','filled')
% scatter(C(3,1),C(3,2),circSize,'r','filled')
% scatter(C(4,1),C(4,2),circSize,'g','filled')
% end
% 
% if(countR == 3)
% scatter(C(1,1),C(1,2),circSize,'g','filled')
% scatter(C(2,1),C(2,2),circSize,'b','filled')
% scatter(C(3,1),C(3,2),circSize,'y','filled')
% scatter(C(4,1),C(4,2),circSize,'r','filled')
% end
% 
% %PLOTTING TOP CORNERS OF CUBE
% 
% 
% 
% %LENA TIME
% %fitgeotrans(movingPoints,fixedPoints,'projective');
% %Lena is 512 by 512
% 
% % if (countR == 0)
% %     L = [1 1;  512 1; 512 512; 1 512;];
% % end
% % if (countR == 1)
% %     L = [1 512; 1 1; 512 1; 512 512];
% % end
% % if (countR == 2)
% %     L = [512 512; 1 512; 1 1; 512 1;];
% % end
% % if (countR == 3)
% %     L = [512 1; 512 512; 1 512; 1 1;];
% % end
% % HH = fitgeotrans([L(:,1) L(:,2)],[C(:,1) C(:,2)],'projective');%geometric transformation, x1,y1,x2,y2. 
% % RR=imref2d(size(frame),[1 size(frame,2)],[1 size(frame,1)]);%%reference is same size as frame
% % Lframe = imwarp(Lena,HH,'OUTPUTVIEW',RR);%create warped image using the H found
% % %imshow(Lframe)
% % 
% % [M,N] = size(Lframe);
% % %Lena homographized image is same size as frame, but everything is black
% % %but lena, so go thourhg and if pixel is black change it to color of scene,
% % %or color in frame
% % for i=1:1080
% %    for j=1:1920
% %       if(Lframe(i,j,1) == 0 && Lframe(i,j,2) == 0 && Lframe(i,j,3) == 0)   % if black change it to actual scene
% %          Lframe(i,j,1) = frame(i,j,1);
% %          Lframe(i,j,2) = frame(i,j,2);
% %          Lframe(i,j,3) = frame(i,j,3);
% %       end
% %    end
% % end
% % 
% % %title(nnn)
% % %imshow(Lframe) %Final Image of lena on scene
% 
% %cube
% H5 = fitgeotrans([UnitQ(:,1) UnitQ(:,2)],[C(:,1) C(:,2)],'projective');
% Kinv = inv(K);
% R12t = Kinv*H1%H5.T;
% r1 = R12t(:,1);
% r2 = R12t(:,2);
% t = R12t(:,3);
% r3 = cross(r1,r2);
% Rt = [r1 r2 r3 t];
% P = K*Rt;
% %define world coord
% if (H1(1,1) > 0)
% xw  = [0,0,-20,1]';
% xw2 = [1,0,-20,1]';
% xw3 = [1,1,-20,1]';
% xw4 = [0,1,-20,1]';
% end
% 
% if (H1(1,1) < 0)
% xw  = [0,0,20,1]';
% xw2 = [1,0,20,1]';
% xw3 = [1,1,20,1]';
% xw4 = [0,1,20,1]';
% end
% %get c coord
% xcO  = P*xw;
% xc2 = P*xw2;
% xc3 = P*xw3;
% xc4 = P*xw4;
% xc  = xcO/xcO(3,1);%1 for last term
% xc2 = xc2/xc2(3,1);
% xc3 = xc3/xc3(3,1);
% xc4 = xc4/xc4(3,1);
% H1;
% hold on
% scatter(xc(1,1),xc(2,1),60,'c','filled')
% scatter(xc2(1,1),xc2(2,1),60,'c','filled')
% scatter(xc3(1,1),xc3(2,1),60,'c','filled')
% scatter(xc4(1,1),xc4(2,1),60,'c','filled')
% 
% %After all corners plotted
% %Wanna plot lines between point 1,2   2,3    3,4   4,1 to get rect of tag
% Line12x = [C(1,1) C(2,1)];
% Line12y = [C(1,2) C(2,2)];
% Line23x = [C(2,1) C(3,1)];
% Line23y = [C(2,2) C(3,2)];
% Line34x = [C(3,1) C(4,1)];
% Line34y = [C(3,2) C(4,2)];
% Line41x = [C(4,1) C(1,1)];
% Line41y = [C(4,2) C(1,2)];
% 
% %Plot lines between upper corners
% Line56x = [xc(1,1) xc2(1,1)];
% Line56y = [xc(2,1) xc2(2,1)];
% Line67x = [xc2(1,1) xc3(1,1)];
% Line67y = [xc2(2,1) xc3(2,1)];
% Line78x = [xc3(1,1) xc4(1,1)];
% Line78y = [xc3(2,1) xc4(2,1)];
% Line85x = [xc4(1,1) xc(1,1)];
% Line85y = [xc4(2,1) xc(2,1)];
% 
% 
% %Vertical Lines
% V1x = [C(1,1) xc(1,1)];
% V1y = [C(1,2) xc(2,1)];
% V2x = [C(2,1) xc2(1,1)];
% V2y = [C(2,2) xc2(2,1)];
% V3x = [C(3,1) xc3(1,1)];
% V3y = [C(3,2) xc3(2,1)];
% V4x = [C(4,1) xc4(1,1)];
% V4y = [C(4,2) xc4(2,1)];
% 
% % %PLOTTING THE LINES
% lineWidth = 2;
% hold on
% plot(Line12x,Line12y,'k','LineWidth',lineWidth);
% plot(Line23x,Line23y,'k','LineWidth',lineWidth);
% plot(Line34x,Line34y,'k','LineWidth',lineWidth);
% plot(Line41x,Line41y,'k','LineWidth',lineWidth);
% 
% plot(Line56x,Line56y,'m','LineWidth',lineWidth);
% plot(Line67x,Line67y,'m','LineWidth',lineWidth);
% plot(Line78x,Line78y,'m','LineWidth',lineWidth);
% plot(Line85x,Line85y,'m','LineWidth',lineWidth);
% 
% plot(V1x,V1y, 'g','LineWidth',lineWidth);
% plot(V2x,V2y, 'g','LineWidth',lineWidth);
% plot(V3x,V3y, 'g','LineWidth',lineWidth);
% plot(V4x,V4y, 'g','LineWidth',lineWidth);
% title(nnn)
% %figure
%We say coner of tag in worldcoordinate is 0,0 1,0 ,1,1 0,1 
%We get homography for that
%Use that to get P
%do P*worldcoord to get in camera coord
%since in our world frame we made corner of tag 0,0,0 the top of the 
%cube point is 0,0,-1, since y axis is flipped
%so multiply P*that cube point, get coord of corner in camera pic
filename2 = sprintf('..\\Data\\Tag2\\Frame_%d.jpg',nnn)
imwrite(frame,filename2)

% f = getframe(gca);
% im = frame2im(f);
% writeVideo(NewVid,frame) %Add frame to video
% countRprev = countR;
% runs = runs+1;
% set(gca,'visible','off')
%cla
%close all
end


%close(NewVid);


    
