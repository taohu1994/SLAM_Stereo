function [ disparity ] = read_disparity_image( filename )
%READ_DISPARITY_IMAGE Reads a disparity image.
%   disparity = read_disparity_image( filename )
%       Reads the image with the given filename, and converts it to a
%       proper disparity image.

%Read it.
disparity = imread(filename); 
%convert the grayscale image to the actual disparity values.
disparity = (double(disparity-1))/256;

end

