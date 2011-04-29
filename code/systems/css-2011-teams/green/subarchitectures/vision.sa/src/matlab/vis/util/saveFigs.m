function saveFigs(figs,fname);
%SAVEFIGS  Save figures.
%	saveFigs(figs,fname)

nin=nargin;
if nin==1 %just filename => save all figures
   fname=figs;
   numfigs=figure;
   close(numfigs);
   numfigs=numfigs-1;
   figs=[1:numfigs];
end;   

nfigs=length(figs);
for i=1:nfigs
   figure(figs(i));
   wbdf=get(figs(i),'WindowButtonDownFcn');
   %if ~isempty(wbdf) & wbdf=='rotate3d(''down'')'
   if ~isempty(wbdf) & isequal(wbdf,'rotate3d down')
      Rot3d(i)=1;
      rotate3d off;
   else 
      Rot3d(i)=0;  
   end;     
   if ~isempty(wbdf) & isequal(wbdf,'zoom down')
      Zm(i)=1;
      zoom off;
   else 
      Zm(i)=0;
   end;     
   cmd=['print' ' -dmfile -f' num2str(figs(i)) ' ' fname num2str(i)];
   eval(cmd); 
   if Rot3d(i)==1
      rotate3d on;
   end;   
   if Zm(i)==1
      zoom on;
   end;   
end;
save(fname,'Rot3d','Zm');

