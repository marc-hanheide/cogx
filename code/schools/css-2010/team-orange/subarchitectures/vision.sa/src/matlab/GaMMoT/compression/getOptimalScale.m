function [ a, b ]= getOptimalScale( f0,HellError )
 
% f0.HellError = HellError ;
% C0 = sum(f0.weights'.*f0.covariances) ;
% f0.C0 = C0 ;
% 
% a_initial = 0.5 ;  
% options = optimset('Display','on','TolFun',0.0001,'TolX',0.01,'LargeScale','on');
% % a = lsqnonlin(@a_function,a_initial,[0],[10],options,f0) ;
% a = fminbnd(@(a) a_function(a,f0),0,10,options) ;
% 
% 
% a = f0.C0*a ;
% b = [] ;
% if( nargout == 2 )
%     fx = f0 ;
%     fx.covariances = f0.covariances + a ;
%     b = uHellinger( f0, fx ) ; 
% end
% 
% if abs(b - f0.HellError) > 0.05 
%     
%    dfsg = 56 ; 
% end
%  
% 
% function F = a_function(a,f0)
% 
% fx = f0 ;
% fx.covariances = f0.covariances + f0.C0*a ; 
% 
% F = 100*(f0.HellError- uHellinger( f0, fx ))^2 ; %(a-2)^2 ; %(f0.HellError - uHellinger( f0, fx ))^2 ; (f0.HellError- uHellinger( f0, fx )).^2*100 ; %
% 
% [a, F]


% a = 1.2*(1 - exp(-(length(f0.weights))/6)+1.1) / 1.4 ;
% fx = f0 ;
% fx.covariances = fx.covariances*a ;
% b = MCHellinger( fx, f0, 100000 ) ;
% 
% b2 = uHellinger( f0, fx, [], 0 ) ; b = [b, b2] ;
 
f0.HellError = HellError ;
a_initial = 1.5 ; % was 2.1 this is theoretical value for single gaussian at HellError=0.1
options = optimset('Display','off','TolFun',0.001,'TolX',0.1,'LargeScale','on');
% a = lsqnonlin(@a_function,a_initial,[1],[7],options,f0) ;
a = fminbnd(@(a) a_function(a,f0),1,7,options) ;

b = [] ;
if( nargout == 2 )
    fx = f0 ;
    fx.covariances = fx.covariances*a ;
    b = uHellinger( f0, fx ) ; 
end

if ( abs(b-f0.HellError) > 0.1 )
    fsg = 67 ;
end

function F = a_function(a,f0)

fx = f0 ;
fx.covariances = fx.covariances*a ; 

F = 100*(f0.HellError - uHellinger( f0, fx ))^2 ; 