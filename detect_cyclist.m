function detect_cyclist()
%SELECT_CYCLIST select points on the cyclsit to "detect" him.


subfolder = @(base, sub) [base '/' sub];
merge_file = @(dir) subfolder(dir.folder, dir.name);


img_folder = 'images/left';
disp_folder = 'images/disp_gray';

img_files = dir(img_folder);
img_files = img_files(3:end);
disp_files = dir(disp_folder);
disp_files = disp_files(3:end);

u = zeros(1, numel(img_files));
v = zeros(1, numel(img_files));
figure(1); clf;
for k = 1:numel(img_files)
    image = imread(merge_file(img_files(k)));
    disparity = read_disparity_image(merge_file(disp_files(k)));
    imshow(image .* uint8(repmat(disparity > 2, 1,1,3)))
    
    [u(k), v(k)] = ginput(1);
end

save('detections', 'u', 'v');
end