function [ a, b ]= getOptimalScale2( f0, f1, HellError, maxInflation )

if nargin < 3
    maxInflation = 7 ;
end
 
F.f0 = f0 ; F.f1 = f1 ; F.HellError = HellError ;
a_initial = 1.5 ; % was 2.1 this is theoretical value for single gaussian at HellError=0.1
options = optimset('Display','off','TolFun',0.001,'TolX',0.1,'LargeScale','on');
% a = lsqnonlin(@a_function,a_initial,[1],[7],options,f0) ;
a = fminbnd(@(a) a_function(a,F),1,maxInflation,options) ;

b = [] ;
if( nargout == 2 )
    fx = f0 ;
    fx.covariances = fx.covariances*a ;
%     b = uHellinger(  fx, f1 ) ; 
        b = uHellingerJointSupport(  fx, f1 ) ;%- uHellinger(  fx, f1 ) ; 
end

if ( abs(b-F.HellError) > 0.1 )
    fsg = 67 ;
end

function Err = a_function(a,F)

fx = F.f0 ;
fx.covariances = fx.covariances*a ; 

% Err = 100*(F.HellError - uHellinger(fx,F.f1 ))^2 ; 
Err = 100*(F.HellError - uHellingerJointSupport(fx,F.f1 ))^2 ;
