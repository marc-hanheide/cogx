function pdf = get_1d_OptimalKDE( data ) 
%
% Matej Kristan (2007)
%
% Estimates the optimal bandwidth from data using solve-the-equation
% plug-in method, and creates a KDE estimate.
%
usefast = 0 ;

r = rows( data ) ; 
if ( r > 1 )
    data = data' ;
end
len =length(data) ;

if usefast == 0 
    h_1d = slow_univariate_bandwidth_estimate_STEPI(length(data),data) ;
else
    epsil = 1e-3 ;
    h_1d = fast_univariate_bandwidth_estimate_STEPI(length(data),data,epsil) ;
end
pdf.covariances = ones(len,1)*h_1d^2 ;
pdf.mu = data ;
pdf.weights = ones(1,len)/len ;
