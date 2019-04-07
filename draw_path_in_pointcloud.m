function draw_path_in_pointcloud(camera, disparity, image, varargin)
%DRAW_PATH_IN_POINTCLOUD draw a pointcloud together with a path.
%
%    draw_path_in_pointcloud(camera, disparity, image, path)
%       Draws a pointcloud, and a path. The path is expected to be 3xN.
%
%    draw_path_in_pointcloud(camera, disparity, image, path1, path2,...)
%       Draws a pointcloud, and path1 up to pathN. The paths are expected to be 3xN.
%

% Create limit function to only draw the current street up to a distance of
% 50 meters.
limitfun = @(distance, points) distance < 50 & abs(points(2,:)) < 5;
%create point cloud and show it.
pc = get_pointcloud(camera, disparity, image, limitfun);
pcshow(pc, 'MarkerSize', 10); hold on;

%Draw each path
for k = 1:numel(varargin)
    positions = varargin{k};
    plot3(positions(1,:), positions(2,:), positions(3,:)+0, 'LineWidth', 3)
end

end