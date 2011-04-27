function threshold = getNonLinPdfThreshold( nlfun, alpha )
%
% Matej Kristan (2007)
%
% calculates threshold such that integral over thresholded distribution 
% is alpha percent of the entire integral.
%

P = sum(nlfun) ;
len = length(nlfun) ;
c = 0 ;
I = -1 ;
for i = 1 : len
    c = c + nlfun(i) ;
    if c > alpha*P
        I = i ;
        if I < 1 I = 1 ; end
        break ;
    end    
end

threshold = nlfun(I) ;


