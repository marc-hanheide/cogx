function [ a, b ]= getOptimalScaleToRef( f0, f_ref )

K.f_ref = f_ref ;
K.f0 = f0 ;

a_initial = 1.6 ;  
   
options = optimset('Display','off','TolFun',0.001,'TolX',0.0001,'LargeScale','on');
a = lsqnonlin(@b_function,a_initial,[1.0],[7],options,K) ;
 

b = [] ;
if( nargout == 2 )
    f0.covariances = f0.covariances*a ;
    b = uHellinger( f_ref, f0 ) ; 
end
 

function F = b_function(a,K)
 
f0 = K.f0 ;
f0.covariances = K.f0.covariances*a ; 

F = exp(uHellinger( K.f_ref, f0 )) ; 