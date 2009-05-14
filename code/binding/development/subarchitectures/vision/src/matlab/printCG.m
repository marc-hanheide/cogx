function printCG(mCG,Fnames, CGnames)

fprintf('Detected concept groups:\n');
for i=1:length(mCG)
   fprintf(['CG' num2str(i) '(' Fnames(mCG(i).Fb,:) '): ']);
   for j=1:length(mCG(i).C) 
      fprintf([num2str(CGnames(mCG(i).C(j),:)) ' ']); 
   end;
   fprintf('\n');
end;   
