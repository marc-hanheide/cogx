function b=bin(b0,thr)

if nargin<2
   minb=double(min(b0(:)));
   maxb=double(max(b0(:)));
   thr=minb+2*(maxb-minb)/3;
end;
b=b0>thr;