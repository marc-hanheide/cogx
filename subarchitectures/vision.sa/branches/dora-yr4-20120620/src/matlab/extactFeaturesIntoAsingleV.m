function [AP,CF,MOK,F] = extactFeaturesIntoAsingleV()

global Data ;
global Dirs currMode ;

confFile='config/cogLearn.config';
loadConfig(confFile);
Dirs.images=[Dirs.cogLearn 'files/images/']';
Dirs.asv=[Dirs.cogLearn 'files/asv/'];


upbnd = 0.7 ;
lobnd = 0.66 ;

[AP,CF,MOK]=readAPs(1:876,10) ;

for i = 1 : Data.numImgs 
    Data.currImg = i ;
    [x,b,pt3d]=OSinterface ;
    f=FEinterface(x,b,pt3d) ;
    F(:,i) = f ;    
 
    if f(6) > upbnd && sum(AP(9:10,i)) > 0
        AP(9:10,i) = 0 ;
        AP(10,i) = 1 ;
    end
    
    if f(6) < lobnd && sum(AP(9:10,i)) > 0
        AP(9:10,i) = 0 ;
        AP(9,i) = 1 ;
    end
    
    if f(6) > lobnd && f(6) < upbnd 
        AP(9:10,i) = 0 ;
    end        
end
 