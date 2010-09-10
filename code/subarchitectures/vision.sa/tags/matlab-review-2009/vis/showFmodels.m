function showFmodels(mC,Fnames,Cnames,fn,hp)
%showLmodels(mC,Fnames,Cnames,hp)
%Show learned models

if nargin<4
   fn=1;
end;   
if nargin<5
   hp=gca;
end;

color='rgbycmk';

cs=find([mC.Fb]==fn);

for i=cs
      showDecPdfC(mC(i),color((mC(i).name)),hp);
      hold on;
end
hold off;

title(hp,Fnames(fn,:));
legend(Cnames([mC(cs).name],:));
alim=axis(hp);
alim(4)=40;
set(hp,'Ylim',alim(3:4));
set(hp,'Xlim',[0 1]);


