function displayGD(mC);

if getc(mC,'numSC')>=2
   fh=figure('Visible','Off');
   c1=mC{1}.cummulative_feat_costs;
   c2=mC{2}.cummulative_feat_costs;
   if ~isempty(c1)&&~isempty(c2)
      bar([c1;c2]');
   end
   displayG(fh,'GD');
end

