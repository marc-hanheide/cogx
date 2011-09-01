% Author: Markus Bader 2010
% email: markus.bader@tuwien.ac.at

clear variables
nao = NaoQi('192.168.1.214',9559);

Irgb = nao.getCameraImage(2, 11, 5, 0);
figure( 1); hold off; imshow(Irgb); hold on; 
