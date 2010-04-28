function e = gaussUnlearn( f, r )
% Matej Kristan 2007
%
% Gaussian Unlearning of Gaussian distribution
% f ... input distribution
% g ... unlearning example distribution
% e ... resulting distribution

% search for maximum in the unlearning example
[r_x_max r_y_max]= findGlobalMaximum(r) ;

% get new mixture
e = getUnlearning(f, r, r_x_max) ;
