function showFS(mFS,mC,Fnames,Cnames,hp)

if nargin<5
   hp=gcf;
end;

warning off;

[numC,numF]=size(mFS.Fmeans);

minfs=min(mFS.Fmeans-9*mFS.Fvars,[],1);
maxfs=max(mFS.Fmeans+9*mFS.Fvars,[],1)+.00001;

for i=1:numC
   for j=1:numF
      ha=subplot(numC,numF,(i-1)*numF+j,'Parent',hp);
      showGauss(mFS.Fmeans(i,j),mFS.Fvars(i,j),[minfs(j),maxfs(j)],ha);
      set(ha,'ytick',[]);
      set(ha,'xtick',[]);
      if i==1
          title(ha, Fnames(j,:));
      end
      if j==1
          ylabel(ha, Cnames(mC(i).name,:));
      end
   end
end

warning on;
