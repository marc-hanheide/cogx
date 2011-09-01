function [imgSeg tl br cen area col hist] = huttenlocher(imgSrc, sigma, k, min_size, random_color)
% The LD_LIBRARY_PATH has to be set
% export LD_LIBRARY_PATH=$V4R_DIR/lib:$LD_LIBRARY_PATH
% Options:
%   imgSrc
%   sigma
%   k
%   min_size
%   random_color on=1 off=0
% Returns:
%   imgSeg
%   tl topleft 2 x NrOfSegmetns
%   br bottomright 2 x NrOfSegmetns
%   cen center 2 x NrOfSegmetns
%   area area 1 x NrOfSegmetns
%   col color 4 x NrOfSegmetns
%   his color historam  16 x NrOfSegmetns
%
% Author: Markus Bader
% Version: 0.1

[imgSeg tl br cen area col hist] = mex_huttenlocher(imgSrc, sigma, k, min_size, random_color);
