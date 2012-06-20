function roi=showROIs(imgIdx,imgDir,ipf,imgName,maskName)
%roi=showROIs(imgIdx,imgDir,ipf,imgName,maskName)

if nargin<2
   imgDir='';
end;   
if nargin<3
   ipf=20;
end;
if nargin<4
   imgName='img';
end;   
if nargin<5
   maskName='msk';
end;   

N=length(imgIdx);

numf=floor(N/ipf);
if mod(N,ipf)>0
   numf=numf+1;
end;

allcfigs=[];
for fi=1:numf

   if fi==numf && mod(N,ipf)>0
      idxs=(fi-1)*ipf+1:(fi-1)*ipf+mod(N,ipf);
   else   
      idxs=(fi-1)*ipf+1:fi*ipf;
   end;   

   cfigs=figure;
   allcfigs=[allcfigs,cfigs];

   figs=[];
   for i=idxs

%      imgfile=[imgDir imgName num2str(imgIdx(i),'%03d') ,'.jpg'];
%      maskfile=[imgDir maskName num2str(imgIdx(i),'%03d') ,'.jpg'];
      imgfile=[imgDir imgName num2str(imgIdx(i),'%03d') ,'.png'];
      maskfile=[imgDir maskName num2str(imgIdx(i),'%03d') ,'.png'];

      x=imread(imgfile);
      b=imread(maskfile);
      bb=getBoundingBox(b);
      roi=x(bb(2,1):bb(2,2),bb(1,1):bb(1,2),:);

      cfig=figure;
      imagesc(roi);
      axis off;
      axis image;
      title(num2str(imgIdx(i)));
      figs=[figs cfig];
   end

   figure(cfigs);
   compoundFigs(figs);
   closefigs(figs);

end

comparefigs(allcfigs,2);
