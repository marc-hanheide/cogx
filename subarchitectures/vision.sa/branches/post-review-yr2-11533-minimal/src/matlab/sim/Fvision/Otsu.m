% This function calculates the threshold for the binarisation of 
% images by the OTSU technique.
%
% [threshold,Hs] = otsu(ima);
%	ima 	: input image
%  threshold: optimal threshold
%  Hs	: value of the criterion to maximise for each value of s

function [threshold,Hs] = Otsu(ima)

% First get the histogram
LHist = 256;	% nb of levels of the hsitogram
[hist,binloc] = imhist(ima,LHist);
% Make it a probability:
s = size(ima);
hist = hist/(s(1)*s(2));
% Second calculates the criterion

%Init
Pinf = 0;  	% Pi of the formula
Psup = 1;	% Ps of the formula
sinf = 0;	% Min possible s;
muinf = 0;	% mu_i of the formula
ssup = (0:LHist-1)*hist;
Hs = zeros(1,LHist);
% Calculus
for s = 0:LHist-1
   % proportions
   Pinf = Pinf+hist(s+1);
   Psup = 1-Pinf;	% As Pinf+Psup =1;
   % Local means
   sinf = sinf+s*hist(s+1);	% Numerator for mu_i
   ssup = ssup-s*hist(s+1);	% Numerator for mu_s
   if (Pinf ~=0)
      muinf = sinf/Pinf;
   else	
      muinf = 0;
   end	
   if (Psup~=0)
      musup = ssup/Psup;
   else
      musup = 0;
   end	
   Hs(s+1) = Pinf*Psup*(muinf-musup)^2;
end
thrBin = find(Hs == max(Hs));	% optimal threshold
if length(thrBin>1) thrBin=round((thrBin(end)+thrBin(1))/2); end;
threshold=binloc(thrBin);

return;

clf;
plot(Hs);
hold on;
plot(thrBin,0:max(Hs),'-r');
disp('Press a key to see the resulting image');
pause;
hold off;
imagesc(ima>threshold);
colormap(gray);