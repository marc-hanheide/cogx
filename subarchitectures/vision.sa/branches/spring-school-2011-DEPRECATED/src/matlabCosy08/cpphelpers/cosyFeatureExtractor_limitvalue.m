%% this->eval("b = double(b);");
%% this->eval("b(b > 1) = 1;");
function [newb] = cosyFeatureExtractor_limitvalue(b, maxval) 
   newb=double(b);
   newb(newb>maxval)=maxval;
end

