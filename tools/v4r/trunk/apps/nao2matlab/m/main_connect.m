% Author: Markus Bader 2010
% email: markus.bader@tuwien.ac.at

%connect to robot
clear variables

%nao = NaoQi('127.0.0.1',9559);
nao = NaoQi('192.168.1.202',9559);
%nao = NaoQi('192.168.1.51',9559);
nao.setStiffnesses('Body',1);
%nao.stiffnessInterpolation('HeadYaw', 0, 1, 'HeadPitch', 0,  1);
pause(1)
nao.walkTo(0.02,  0, 0, 1);
active = nao.walkIsActive
nao.getPosition('CameraBottom', 1, true)
nao.setAngles('HeadYaw', 0, 'HeadPitch', 0.3, 0.2, 1);
%nao.setStiffnesses('Body',0.3);
%nao.stiffnessInterpolation('HeadYaw', 0, 1, 'HeadPitch', 0,  1);

nao.close;
