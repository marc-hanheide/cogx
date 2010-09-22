function answ=LRinterface(req,avw,f)

disp(['LRinterface: I will process [' num2str(req) '] [' num2str(avw(:)') ']!']);

global mC Params Coma Figs
readConstants;

global f1;
f1=f;

switch req
   case 1 %recognize
      avu=ODKDErec(f,mC); %recognize AVs considering current models
      answ=avu;
      rCpcx=cc2c(answ,'trim');
      ansQl=qnt2ql(rCpcx,Params.THRs);
      ansYes = lf2sfa(ansQl, ANSyes);
      ansPy = lf2sfa(ansQl, ANSpy);
      disp(['Recognised: ',idx2name(ansYes,Coma.Cnames)]);
      showRec(ansYes,ansPy,rCpcx,f);
      displayTR(ansYes,ansPy,avu);
      displayG(Figs.LRguiR.main,'GR');
   case 2 %update
      avwp=avw(find(avw(:,2)>0),:);
      avwn=avw(find(avw(:,2)<0),:);%avwn=avwn(:,1);

      if ~isempty(avwp)
         c=avw2snf(avwp,Coma.SCC);
         mC=ODKDEupdate(f,c,mC);
      end
      if ~isempty(avwn)
         c=avw2snf(avwn,Coma.SCC);
         mC=ODKDEunlearn(f,c,mC);
      end
      answ=1;%OK
      LRvisUpdate;
%      LRevalUpdate;
   otherwise %do nothing
      answ=0;%OK nothing
end;