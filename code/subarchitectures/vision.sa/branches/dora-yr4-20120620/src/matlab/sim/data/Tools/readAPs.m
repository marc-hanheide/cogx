function [AP,CF,MOK]=readAPs(idxs,numAV,fdir)

if nargin<3 
   fdir=[]; 
end;

N=length(idxs);
AP=zeros(numAV,N);
CF=zeros(numAV,N);
if nargout==3
   MOK=zeros(1,N);
end;

for i=1:N
   datfile=['D:\Work\Matlab\CogX\DK_code\sim\data\objects_newvis\' 'dat' num2str(idxs(i),'%04d') ,'.mat'];
   load(datfile,'gt');

   for j=1:size(gt.AP,1);
      AP(gt.AP(j,1),i)=1;
      CF(gt.AP(j,1),i)=gt.AP(j,2);
   end;
   if nargout==3
      if isempty(gt.MOK)
         gt.MOK=0;
      end
      MOK(i)=gt.MOK;
   end;

end
