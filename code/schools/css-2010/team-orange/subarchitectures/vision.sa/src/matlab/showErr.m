function showErr(mC,resC,Ft,Ct,sphw)


if nargin<5
   sphw=[3 4];
end   

numC=length(resC.rss);

for i=1:numC
   idxs=find(resC.RS(mC(i).name,:)<1);
   subplot(sphw(1),sphw(2),i);
   hold on;
   lims=axis(gca);

%    idfp=find(Ft(mC(i).Fb,idxs)==0);
%    idfn=find(Ft(mC(i).Fb,idxs)==1);
%    plot(Ft(mC(i).Fb,idxs(idfp)),zeros(size(Ft(mC(i).Fb,idxs(idfp)))),'ms'); %FN
%    plot(Ft(mC(i).Fb,idxs(idfn)),zeros(size(Ft(mC(i).Fb,idxs(idfn)))),'bo'); %FN
   for j=1:length(idxs);
         xi=Ft(mC(i).Fb,idxs(j));
      if Ct(mC(i).name,idxs(j))==1
         plot(Ft(mC(i).Fb,idxs(j)),0,'bo'); %FN
         line([xi;xi],[lims(3);lims(4)],'Color','b');
         %fn=getBel(mC(i),xi)
      else
         plot(Ft(mC(i).Fb,idxs(j)),0,'ms'); %FP
         line([xi;xi],[lims(3);lims(4)],'Color','m');
         %fp=getBel(mC(i),xi)
      end
   end

end;





% 
% function showErr(mC,resC,Ft,spwh)
% 
% 
% if nargin<4
%    sphw=[3 4];
% end   
% 
% numC=length(resC.rss);
% 
% for i=1:numC
%    idxs=find(resC.RS(mC(i).name,:)<1);
%    subplot(sphw(1),sphw(2),i);
%    hold on;
%    %axs=axis;
%    for j=1:length(idxs);
%       plot(Ft(mC(i).Fb,idxs(j)),0,'o');
%    end
%    axis tight
% 
% end;