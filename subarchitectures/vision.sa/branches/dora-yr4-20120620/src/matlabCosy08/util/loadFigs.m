function loadFigs(fname);
%LOADFIGS  Load figures.

load(fname);
numfigs=length(Rot3d);
figs=[1:numfigs];
for i=1:numfigs
   cmd=[fname num2str(i)];
   eval(cmd);
   if Rot3d(i)==1
      rotate3d on;
   end;   
   if Zm(i)==1
      zoom on;
   end;   
end;

