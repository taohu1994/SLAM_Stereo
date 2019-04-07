function [ resPointCloud] = get_pointcloud( camera, disparity, image, limitfun)
%GET_POINTCLOUD Create point cloud. See also compute_3d,
%       read_disparity_image, pointCloud and pcshow. 
%
%    [resPointCloud] = get_pointcloud(camera, disparity, image)
%       Creates a point cloud from camera paramaters, a disparity image and
%       an RGB image.
%    [resPointCloud] = get_pointcloud(camera, disparity, image, limitfun)
%       Creates a point cloud from camera paramaters, a disparity image and
%       an RGB image. Limits the accepted points through LIMITFUN, which takes 
%       The form of a function limitfun(distance, points).
%        
%       Examples:
%       Show only points within a 50 meter radius:
%       limitfun = @(distance, points) distance < 50;
%       [resPointCloud] = get_pointcloud(camera, disparity, image, limitfun)
%
%       Show only points with a positive z value:
%       limitfun = @(distance, points) points(3,:) > 0;
%       [resPointCloud] = get_pointcloud(camera, disparity, image, limitfun)
%
%       Show only points within a 50 meter radius and a positive z value:
%       limitfun = @(distance, points) distance < 50 & points(3,:) > 0;
%       [resPointCloud] = get_pointcloud(camera, disparity, image, limitfun)
%

[h, w] = size(disparity);
[u,v] = meshgrid(1:w,1:h);

%Remove all points that have a negative or extremely small disparity.
goodPoints = disparity(:)>1;
u = u(goodPoints)';
v = v(goodPoints)';

%compute 3d points
points = compute_3d(camera, disparity, u, v);

%If requested, remove some of the points based on limitfun.
if nargin == 4
    distance = sqrt(sum(points.^2,1));
    points(:, ~limitfun(distance, points)) = NaN;
end

%Create point cloud.
image = reshape(image,[],3);
resPointCloud = pointCloud(points', 'Color', double(image(goodPoints,:))/256);
end

