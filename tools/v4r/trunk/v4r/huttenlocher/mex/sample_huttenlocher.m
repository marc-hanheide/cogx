% The LD_LIBRARY_PATH has to be set first
% export LD_LIBRARY_PATH=$V4R_DIR/lib:$LD_LIBRARY_PATH
% export LD_LIBRARY_PATH=${ROS_ROOT}/../stacks/vision_opencv/opencv2/opencv/lib:$LD_LIBRARY_PATH
close all
clear variables
I = imread('tinyphoon.bmp');
figure( 1)
imshow(I)

Is  = huttenlocher(I, 0.7, 200, 50, 1);

figure( 1)
imshow(Is)
