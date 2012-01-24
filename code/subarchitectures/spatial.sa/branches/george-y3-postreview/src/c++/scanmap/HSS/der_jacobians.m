% This file derives various Jacobians

syms xsR ysR asR xrW yrW arW xsW ysW asW xlW ylW alW 

% Pose of sensor in world frame
xsW = xrW + cos(arW)*xsR - sin(arW)*ysR;
ysW = yrW + sin(arW)*xsR + cos(arW)*ysR;
asW = arW + asR;

% Pose of landmark in sensor frame
xlS = cos(asW) * (xlW - xsW) + sin(asW) * (ylW - ysW);
ylS =-sin(asW) * (xlW - xsW) + cos(asW) * (ylW - ysW);
alS = alW - asW;

disp('Differentials of landmark pose in sensor frame w.r.t. robot world pose')
diff(xlS,'xrW')
diff(xlS,'yrW')
diff(xlS,'arW')
diff(ylS,'xrW')
diff(ylS,'yrW')
diff(ylS,'arW')
diff(alS,'xrW')
diff(alS,'yrW')
diff(alS,'arW')

% H(1+(i-1)*3,1) =-cos(arW+asR);
% H(1+(i-1)*3,2) =-sin(arW+asR);
% H(1+(i-1)*3,3) =-sin(arW+asR)*(xlW-xrW-cos(arW)*xsR+sin(arW)*ysR)+cos(arW+asR)*(sin(arW)*xsR+cos(arW)*ysR)+cos(arW+asR)*(ylW-yrW-sin(arW)*xsR-cos(arW)*ysR)+sin(arW+asR)*(-cos(arW)*xsR+sin(arW)*ysR);
% H(2+(i-1)*3,1) = sin(arW+asR);
% H(2+(i-1)*3,2) =-cos(arW+asR);
% H(2+(i-1)*3,3) =-cos(arW+asR)*(xlW-xrW-cos(arW)*xsR+sin(arW)*ysR)-sin(arW+asR)*(sin(arW)*xsR+cos(arW)*ysR)-sin(arW+asR)*(ylW-yrW-sin(arW)*xsR-cos(arW)*ysR)+cos(arW+asR)*(-cos(arW)*xsR+sin(arW)*ysR); 
% H(3+(i-1)*3,1) = 0;
% H(3+(i-1)*3,2) = 0;
% H(3+(i-1)*3,3) = -1;
 
disp('Differentials of sensor pose in landmark frame of w.r.t. landmark world position')
diff(xlS,'xlW')
diff(xlS,'ylW')
diff(xlS,'alW')
diff(ylS,'xlW')
diff(ylS,'ylW')
diff(ylS,'alW')
diff(alS,'xlW')
diff(alS,'ylW')
diff(alS,'alW')

% H(1+(i-1)*3,1+index_lm(p,1)*3) = cos(arW + asR);
% H(1+(i-1)*3,2+index_lm(p,1)*3) = sin(arW + asR);
% H(1+(i-1)*3,3+index_lm(p,1)*3) = 0;
% H(2+(i-1)*3,1+index_lm(p,1)*3) = -sin(arW + asR);
% H(2+(i-1)*3,2+index_lm(p,1)*3) = cos(arW + asR);
% H(2+(i-1)*3,3+index_lm(p,1)*3) = 0;
% H(3+(i-1)*3,1+index_lm(p,1)*3) = 0;
% H(3+(i-1)*3,2+index_lm(p,1)*3) = 0;
% H(3+(i-1)*3,3+index_lm(p,1)*3) = 1; 

syms xlR ylR alR

% Landmark pose in sensor frame
xlS = cos(asR) * (xlR - xsR) + sin(asR) * (ylR - ysR);
ylS =-sin(asR) * (xlR - xsR) + cos(asR) * (ylR - ysR);
alS = alR - asR;

disp('Differentials of landmark pose in sensor frame w.r.t. landmark pose in robot frame')
diff(xlS, 'xlR')
diff(xlS, 'ylR')
diff(xlS, 'alR')
diff(ylS, 'xlR')
diff(ylS, 'ylR')
diff(ylS, 'alR')
diff(alS, 'xlR')
diff(alS, 'ylR')
diff(alS, 'alR')

% H_roboc(1+(i-1)*3,1+index_lm(p,1)*3) = cos(asR);
% H_roboc(1+(i-1)*3,2+index_lm(p,1)*3) = sin(asR);
% H_roboc(1+(i-1)*3,3+index_lm(p,1)*3) = 0;
% H_roboc(2+(i-1)*3,1+index_lm(p,1)*3) =-sin(asR);
% H_roboc(2+(i-1)*3,2+index_lm(p,1)*3) = cos(asR);
% H_roboc(2+(i-1)*3,3+index_lm(p,1)*3) = 0;
% H_roboc(3+(i-1)*3,1+index_lm(p,1)*3) = 0;
% H_roboc(3+(i-1)*3,2+index_lm(p,1)*3) = 0;
% H_roboc(3+(i-1)*3,3+index_lm(p,1)*3) = 1;

% Derives the Jacobanof the inverse compound operation

disp('icompound Jacobian')

syms tx ty ta
itx = -cos(ta)*tx-sin(ta)*ty;
ity =  sin(ta)*tx-cos(ta)*ty;
ita = -ta;

diff(itx, 'tx')
diff(itx, 'ty')
diff(itx, 'ta')
diff(ity, 'tx')
diff(ity, 'ty')
diff(ity, 'ta')
diff(ita, 'tx')
diff(ita, 'ty')
diff(ita, 'ta')

% J(0,0) = -cos(ta);
% J(0,1) = -sin(ta);
% J(0,2) = sin(ta)*tx-cos(ta)*ty;
% J(1,0) = sin(ta);
% J(1,1) = -cos(ta);
% J(1,2) = cos(ta)*tx+sin(ta)*ty;
% J(2,0) = 0;
% J(2,1) = 0;
% J(2,2) = -1;

disp('Jacobian of compound operator')

syms v1x v1y v1a v2x v2y v2a

cx = v1x + cos(v1a)*v2x - sin(v1a)*v2y;
cy = v1y + sin(v1a)*v2x + cos(v1a)*v2y;
ca = v1a + v2a;

diff(cx, 'v1x')
diff(cx, 'v1y')
diff(cx, 'v1a')
diff(cx, 'v2x')
diff(cx, 'v2y')
diff(cx, 'v2a')
diff(cy, 'v1x')
diff(cy, 'v1y')
diff(cy, 'v1a')
diff(cy, 'v2x')
diff(cy, 'v2y')
diff(cy, 'v2a')
diff(ca, 'v1x')
diff(ca, 'v1y')
diff(ca, 'v1a')
diff(ca, 'v2x')
diff(ca, 'v2y')
diff(ca, 'v2a')

% J(0,0) = 1;
% J(0,1) = 0;
% J(0,2) = -sin(v1a)*v2x-cos(v1a)*v2y;
% J(0,3) = cos(v1a);
% J(0,4) = -sin(v1a);
% J(0,5) = 0;
% J(1,0) = 0;
% J(1,1) = 1;
% J(1,2) = cos(v1a)*v2x-sin(v1a)*v2y;
% J(1,3) = sin(v1a);
% J(1,4) = cos(v1a);
% J(1,5) = 0;
% J(2,0) = 0;
% J(2,1) = 0;
% J(2,2) = 1;
% J(2,3) = 0;
% J(2,4) = 0;
% J(2,5) = 1;
