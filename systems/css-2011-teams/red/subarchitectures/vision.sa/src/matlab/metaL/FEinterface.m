function f=FEinterface(x,b,pt3d)

global Params Data

x=uint8(x);
f=extAPfeatures(x,b,Params.FV,pt3d);
fprintf('  F= ');fprintf('%3.3g  ',f);fprintf('\n');
 
showROI(x,b,f, pt3d);
% 
% subplot(1,3,3) ; 
% hold off ;
% plot3(pt3d(:,1), pt3d(:,2), pt3d(:,3), '*r') ;
% msg = sprintf('%d Shvals: %1.3g, %1.3g, %1.3g', Data.currImg , f(4), f(5), f(6)) ; title(msg) ;
% menufigs ; axis equal ;

% pause() ;