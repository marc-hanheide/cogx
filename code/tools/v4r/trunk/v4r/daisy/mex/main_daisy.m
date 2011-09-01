% The LD_LIBRARY_PATH has to be set first
% export LD_LIBRARY_PATH=$V4R_DIR/lib:$LD_LIBRARY_PATH
% export LD_LIBRARY_PATH=$ROS_ROOT/../stacks/vision_opencv/opencv2/opencv/lib:$LD_LIBRARY_PATH

close all;
fileA = '01a.jpg'
fileB = '01b.jpg'

Ia = imread(fileA);
Ib = imread(fileB);

Da = daisy(Ia);
Db = daisy(Ib);

GuiFindCorr(Ia, Ib, Da, Db);

