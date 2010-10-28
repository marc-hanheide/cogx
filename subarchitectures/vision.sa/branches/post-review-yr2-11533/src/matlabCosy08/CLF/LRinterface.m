function answ=LRinterface(req,avw,f)

disp(['LRinterface: I will process [' num2str(req) '] [' num2str(avw(:)') ']!']);

global mAV mDA mFS currMode;
readConstants;

switch req
   case 1 %recognize
      %avu=MVBFrecAV(f,mAV); %recognize AVs considering current models
      %avu=MVBFrec(f,mAV,mFS); %recognize AVs considering current models
      avu=KDBFrec(f,mAV,mFS); %recognize AVs considering current models
      answ=avu;
      if currMode.qnt2qlD==0
         ansQl = qnt2ql(answ, currMode.THRs);
      else   
         ansQl = qnt2qlD(answ, currMode.THRs, currMode.CTT);
      end
      ansYes = lf2sfa(ansQl, ANSyes);
      ansPy = lf2sfa(ansQl, ANSpy);
      showRec(ansYes,ansPy,answ,f);
   case 2 %update
      %      [mAV,mDA,mFS]=MVBFupdate(f,avw,mAV,mDA,mFS);

      %[mAV,mDA,mFS]=KDBFupdate(f,avw,mAV,mDA,mFS);
      avwp=avw(find(avw(:,2)>0),:);
      avwn=avw(find(avw(:,2)<0),:);avwn=avwn(:,1);

      if ~isempty(avwp)
         [mAV,mDA,mFS]=KDBFupdate(f,avwp,mAV,mDA,mFS);
      end
      if ~isempty(avwn)
         [mAV,mDA,mFS]=KDBFunlearn(f,avwn,mAV,mDA,mFS);
      end
      answ=1;%OK
      LRvisUpdate;
%      LRevalUpdate;
   otherwise %do nothing
      answ=0;%OK nothing
end;