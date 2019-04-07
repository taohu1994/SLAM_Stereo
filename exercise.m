clc;
close all;
clear
subfolder = @(base, sub) [base '/' sub];
merge_file = @(dir) subfolder(dir.folder, dir.name);
img_folder = 'images/left';
disp_folder = 'images/disp_gray';
img_files = dir(img_folder);
img_files = img_files(3:end);
disp_files = dir(disp_folder);
disp_files = disp_files(3:end);
image = imread(merge_file(img_files(1)));
disparity = read_disparity_image(merge_file(disp_files(1)));
 disparity_end = read_disparity_image(merge_file(disp_files(61)));
 image_end=imread(merge_file(img_files(61)));
load('camera');
%figure(1); clf;
%imshow(image);
%disp('Select some points, finish by pressing enter or esc.');
% [u, v] = getpts();
% points = compute_3d(camera, disparity, round(u)', round(v)');
% 
% f = figure(2); clf; 
% %make background white instead of gray
% f.Color = [1 1 1];
% draw_path_in_pointcloud(camera, disparity, image, points);
% 
%% PART ONE%%
%for all sets of frames f_prev and f_now:
%   find SIFT features in f_prev
%   find SIFT features in f_now
%   compute matches
%   compute 3D points of matches in f_prev
%   compute 3D points of matches in f_now
%   (maybe remove some of the bad matches? e.g. matches with a huge
%   distance between them)
%   find transformation between f_prev and f_now
%
%convert all transformations into path
%draw path in pointcloud
[N,~]=size(img_files);
for ii=2:1:N
   f_prev = single(rgb2gray(imread(merge_file(img_files(ii-1)))));
   f_now = single(rgb2gray(imread(merge_file(img_files(ii)))));
   disparity_prev=read_disparity_image(merge_file(disp_files(ii-1)));
   disparity_now=read_disparity_image(merge_file(disp_files(ii)));
   PT=0; % peak threshold, smaller-more keypoints, default 0
   ET=100; 
   downsample=4;
   [F_prev,d_prev] = vl_sift(f_prev(1:downsample:end,1:downsample:end),'PeakThresh',PT,'EdgeThresh',ET);
   [F_now,d_now] = vl_sift(f_now(1:downsample:end,1:downsample:end),'PeakThresh',PT,'EdgeThresh',ET);
   F_now(1:2,:)=F_now(1:2,:)*downsample;
   F_prev(1:2,:)=F_prev(1:2,:)*downsample;
   [match,scores]=vl_ubcmatch(d_prev,d_now,3);
   if ii==2
   figure(1);
   plotmatches(f_prev,f_now,F_prev,F_now,match,'Stacking','o');
   title('Frist 2 frames SIFT matching')
   end
   [~,NN]=size(match);
   F_prev_filter=[];
   F_now_filter=[];
   for jj=1:1:NN
      F_prev_filter(jj,:)=F_prev(1:2,match(1,jj));
      F_now_filter(jj,:)=F_now(1:2,match(2,jj));
   end
   F_prev_filter=round(F_prev_filter)';
   F_now_filter=round(F_now_filter)';
   x_prev=F_prev_filter(1,:);
   y_prev=F_prev_filter(2,:);
   x_now=F_now_filter(1,:);
   y_now=F_now_filter(2,:);
   %2D pixel feature point to 3D point 
   [point_prev]=compute_3d(camera,disparity_prev,x_prev,y_prev);
   [point_now]=compute_3d(camera,disparity_now,x_now,y_now);
   %delete the NaN data
   plus_buffer=point_prev+point_now;
   point_prev(:,~any(~isnan(plus_buffer), 1))=[];%clear the NaN col
   point_now(:,~any(~isnan(plus_buffer), 1))=[];%clear the NaN col
   %Compute the egomotion transformation matrix [R|T]
   prev_center=mean(point_prev,2);
   now_center=mean(point_now,2);
   point_prev=point_prev-prev_center;
   point_now=point_now-now_center;
   [~,NN]=size(point_prev);
   S=point_now*point_prev';
   [u,d,v]=svd(S);
   R(ii-1,:,:)=(u*v');%Rotation matrix between ii-1,ii frame
   T(ii-1,:)=(now_center-squeeze(R(ii-1,:,:))*prev_center)';%tranlation vector between ii-1,ii frame
end
%produce the culmulative transformation matrix
T_k_cum(1,:,:)=eye(4);%initial transformation matrix
for ii=2:1:N
    T_k(ii-1,:,:)=[squeeze(R(ii-1,:,:)) T(ii-1,:)'; 0 0 0 1];
    T_k_cum(ii,:,:)=squeeze(T_k_cum(ii-1,:,:))*squeeze(T_k(ii-1,:,:));
end
%computering the egomotion vehicle path
path_point(1,:)=[0,0,0,1];%inital coordinate
for ii=2:1:N
   path_point(ii,:)=(inv(squeeze(T_k_cum(ii-1,:,:)))*path_point(1,:)')'; 
end
 f = figure(2); clf; 
 %make background white instead of gray
 f.Color = [1 1 1];
 draw_path_in_pointcloud(camera, disparity, image, path_point(:,1:3)');
 title('The 3D street scene with egomotion path in the first frame scene');
%% PART TWO%%
load 'Tcum_est.mat'
%given the path from part one, or the given path:
%select points on the cyclist in each frame.
%compute the 3D position of points
%create ego-motion compensated version of points
%filter the points using a Kalman filter.
for ii=2:1:N
    path_point(ii,:)=(cell2mat(Tcum_est(ii-1))*path_point(1,:)')';
end
 f = figure(4); clf; 
 %make background white instead of gray
 f.Color = [1 1 1];
 draw_path_in_pointcloud(camera, disparity, image, path_point(:,1:3)');
 title('The 3D street scene with robust vehicle path in the first frame scene');
 load 'detections.mat'
 %transfer the 2d pixels to 3d local world coordinates 
 for ii=1:1:N
   disparity2=read_disparity_image(merge_file(disp_files(ii)));
   point_cyc(ii,:)=compute_3d(camera,disparity2,round(u(ii)),round(v(ii)))';
 end
point_cyc_3d=point_cyc;
point_cyc_3d(:,4)=1;
%transfer the 3d local coordinates points to world coordinate.
for ii=1:1:N
   point_cyc_3d(ii,:)=(inv(squeeze(T_k_cum(ii,:,:)))*point_cyc_3d(ii,:)')';
  % point_cyc_3d(ii,:)=(inv(cell2mat(Tcum_est(ii)))*point_cyc_3d(ii,:)')';
   %point_cyc_3d(ii,:)=(cell2mat(Tcum_est(ii))*point_cyc_3d(ii,:)')';
   %point_cyc_3d(ii,:)=((squeeze(T_k_cum(ii,:,:)))*point_cyc_3d(ii,:)')';
end
 f = figure(6); clf; 
 f.Color = [1 1 1];

draw_path_in_pointcloud(camera, disparity, image, point_cyc_3d(:,1:3)');
 title('The cyclist trajectory in first frame scene');
 figure(7)
 plot3(point_cyc_3d(:,1),point_cyc_3d(:,2),point_cyc_3d(:,3),'*-');
 xlabel('x [m]');
 ylabel('y [m]');
 zlabel('z [m]');
 title('trajectory of cyclist');
figure(8)
plot(point_cyc_3d(:,1),point_cyc_3d(:,2),'*-');
title('Cyclist on x-y plane');
xlabel('longitudinal distance [m]');
ylabel('laternal distance [m]')
%kalman filter with default argument
Q=blkdiag(eye(3),eye(3));
R=eye(3);
x_0=[30;0;0;0;0;0];
P_0=blkdiag(eye(3)*1e2,eye(3));
%system parameters
A=[eye(3),eye(3);zeros(3),eye(3)];
C=[eye(3),zeros(3,3)];
%initial kalman filter
 X_d(1,:)=(A*x_0)';
    P(1,:,:)=A*P_0*A'+Q;
    %error
    e=(point_cyc_3d(1,1:3)'-C*X_d(1,:)')';
    %kalman gain
    K=squeeze(P(1,:,:))*C'*inv((C*squeeze(P(1,:,:))*C'+R));
    X_d(1,:)=(X_d(1,:)'+K*e')';
    P(1,:,:)=squeeze(P(1,:,:))-K*C*squeeze(P(1,:,:));
%filtering the rest data
for ii=2:1:N
    %prediction
    X_d(ii,:)=(A*X_d(ii-1,:)')';
    P(ii,:,:)=A*squeeze(P(ii-1,:,:))*A'+Q;
    %error
    e=(point_cyc_3d(ii,1:3)'-C*X_d(ii,:)')';
    %kalman gain
    K=squeeze(P(ii,:,:))*C'*inv((C*squeeze(P(ii,:,:))*C'+R));
    X_d(ii,:)=(X_d(ii,:)'+K*e')';
    P(ii,:,:)=squeeze(P(ii,:,:))-K*C*squeeze(P(ii,:,:));
end
% kalman filter changing the arguments
model_noisy_gain=1;
measurement_noisy_gain=1e2;
Q=blkdiag(eye(3),eye(3))*model_noisy_gain;
R=eye(3)*measurement_noisy_gain;
x_0=[33;0.2;2.65;0.35;0;0];
P_0=blkdiag(eye(3)*1e2,eye(3));
%system parameters
A=[eye(3),eye(3);zeros(3),eye(3)];
C=[eye(3),zeros(3,3)];
%initial kalman filter
 X(1,:)=(A*x_0)';
    P(1,:,:)=A*P_0*A'+Q;
    %error
    e=(point_cyc_3d(1,1:3)'-C*X(1,:)')';
    %kalman gain
    K=squeeze(P(1,:,:))*C'*inv((C*squeeze(P(1,:,:))*C'+R));
    X(1,:)=(X(1,:)'+K*e')';
    P(1,:,:)=squeeze(P(1,:,:))-K*C*squeeze(P(1,:,:));
%filtering the rest data
for ii=2:1:N
    %prediction
    X(ii,:)=(A*X(ii-1,:)')';
    P(ii,:,:)=A*squeeze(P(ii-1,:,:))*A'+Q;
    %error
    e=(point_cyc_3d(ii,1:3)'-C*X(ii,:)')';
    %kalman gain
    K=squeeze(P(ii,:,:))*C'*inv((C*squeeze(P(ii,:,:))*C'+R));
    X(ii,:)=(X(ii,:)'+K*e')';
    P(ii,:,:)=squeeze(P(ii,:,:))-K*C*squeeze(P(ii,:,:));
end
%%
 f = figure(9); clf; 
 f.Color = [1 1 1];
 draw_path_in_pointcloud(camera, disparity, image, point_cyc_3d(:,1:3)',X(:,1:3)',X_d(:,1:3)');
 title('The filtered and original cyclist trajectory in first frame scene');
 figure(10)
 plot3(point_cyc_3d(:,1),point_cyc_3d(:,2),point_cyc_3d(:,3),'*-');
 hold on
 plot3(X(:,1),X(:,2),X(:,3),'o-');
  hold on
 plot3(X_d(:,1),X_d(:,2),X_d(:,3),'.-');
 hold off
 xlabel('x');  ylabel('y');  zlabel('z');
legend('Original','Optimal value filterd','Default value filterd');
  xlabel('x [m]');
 ylabel('y [m]');
 zlabel('z [m]');
 title('trajectory of cyclist');
figure(11)
plot(point_cyc_3d(:,1),point_cyc_3d(:,2),'*-');
hold on
plot(X(:,1),X(:,2),'o-');
hold on
plot(X_d(:,1),X_d(:,2),'.-');
hold off
title('Cyclist x-y plane');
xlabel('longitudinal distance [m]');
ylabel('laternal distance [m]');
legend('Original','Optimal value filterd','Default value filterd');
figure(12)
plot(point_cyc_3d(:,1),point_cyc_3d(:,3),'*-');
hold on
plot(X(:,1),X(:,3),'o-');
hold on
plot(X_d(:,1),X_d(:,3),'o-');
hold off
title('Cyclist x-z plane');
xlabel('longitudinal distance [m]');
ylabel('vertical distance [m]');
legend('Original','Optimal value filterd','Default value filterd');

