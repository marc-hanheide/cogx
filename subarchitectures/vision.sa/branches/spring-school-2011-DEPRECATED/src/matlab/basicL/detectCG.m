function detectCG(mC,Fnames, Cnames)

numC=length(mC);
usefullF{1}=mC(1).Fb;
k=1;
for i=2:numC
   old=0;
   for j=1:k
      if isequal(mC(i).Fb,usefullF{j})
         old=1;
      end
   end
   if ~old
      k=k+1;
      usefullF{k}=mC(i).Fb;
   end
end
numCG=length(usefullF);
mCG=struct('Fb',[],'C',[]);
for i=1:numCG
   mCG(i).Fb=usefullF{i};
   k=0;
   for j=1:numC
      if isequal(mC(j).Fb,usefullF{i})
         k=k+1;
         mCG(i).C(k)=mC(j).name;
      end
   end
end


fprintf('Detected concept groups:\n');
for i=1:length(mCG)
   fns=Fnames(mCG(i).Fb,:);
   fprintf(['CG' num2str(i) '(' reshape(fns',1,numel(fns)) '): ']);
   for j=1:length(mCG(i).C)
      fprintf([num2str(Cnames(mCG(i).C(j),:)) ' ']);
   end;
   fprintf('\n');
end;