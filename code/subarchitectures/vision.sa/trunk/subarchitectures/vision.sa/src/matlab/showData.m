
NB=400;

for i=1:9%16:17%1:5%numAV

   for j=1:numF

      idxs=find(AVlf(i,:));
      nidxs=rest(1:N,idxs);

      [hav,lav]=hist(F(j,idxs),NB);
      [hnav,lnav]=hist(F(j,nidxs),NB);

      hav=hav/length(F(j,idxs));
      hnav=hnav/length(F(j,nidxs));

      dfigure([attNames(i,:) ': ' Fnames(j,:)]);
      bar(lnav,hnav,'r');hold on;
      bar(lav,hav,'k');

   end

end