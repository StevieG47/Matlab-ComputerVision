%PLOT TRAJECTORY
close all
axisLimit = 90;
%3D PLOT
figure
plot3(path2(:,1),path2(:,2),path2(:,3))
hold on
scatter3(path2(:,1),path2(:,2),path2(:,3),'r','filled')
axis([-2 55 -55 55]);
title('3D Plot of Trajectory')

%2D PLOT
subplot(1,2,1)
%plot(path2(:,3),path2(:,1))
%hold on
scatter(path(:,1),path(:,3),'r','filled')
axis([-5 12 -5 15]);
title('Custom Functions')

%OTher2D PLOT
subplot(1,2,2)
%plot(path2(:,3),path2(:,1))
%hold on
scatter(path4(:,1),path4(:,3),'g','filled')
axis([-5 12 -5 15]);

title('Matlab Functions')