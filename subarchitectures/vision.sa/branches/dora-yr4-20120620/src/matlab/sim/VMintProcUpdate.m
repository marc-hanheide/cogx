function intProc=VMintProcUpdate(step,pl,avw,ansLr)

global Params

readConstants;

if nargin<4, ansLr=[]; end;
ip1={};
asv=0;

if step==1 %from VMprocReq
   if pl(1)==2
      ip1=[ip1;' ';'Get new image!'];
   end;
   if pl(2)==1 %rec.
      ip1=[ip1;'Recognize AVs!'];
   elseif pl(2)==2 %upd
      avs=listAVs(avw);
      ip1=[ip1;['Update with:' avs]];
   end;

else %from VMprocAns

   %beggining of VMprocAns
   if size(ansLr,2)~=1 %from rec.
      
      rCpcx=cc2c(ansLr,'trim');
      ansQl=qnt2ql(rCpcx,Params.THRs);
      
      
      ansYes=lf2sfa(ansQl,ANSyes);
      ansPy=lf2sfa(ansQl,ANSpy);
      if ~isempty(ansYes)
         avsYes=listAVs(ansYes');
         ip1=[ip1;['    Yes:' avsYes]];
      end;
      if ~isempty(ansPy)
         avsPy=listAVs(ansPy');
         ip1=[ip1;['    Py:' avsPy]];
      end;
   end;

   %end of VMprocAns
   if pl(1)<0
      ip1=[ip1;'Answer/ask!'];
   elseif pl(1)==1
      avs=listAVs(avw);
      ip1=[ip1;['Update with:' avs]];
   elseif pl(1)==2
      avs=listAVs(avw);
      ip1=[ip1;['Update with:' avs]];
      ip1=[ip1;'Ask!'];
   end;

   asv=1;

end;

global Figs;
lbipH=Figs.vmipH.lbipH;
intProc=get(lbipH,'String');
intProc=[intProc; ip1];
set(lbipH,'String',intProc);
set(lbipH,'Value',size(intProc,1));

if asv==1
   asvSave;
end;






function avs=listAVs(avw)
global Coma
avs=Coma.avNames(avw(:,1));
for i=1:size(avs,1)
   avs1(i,:)={[' ' cell2mat(avs(i,:))]};
end;
avs=cell2mat(avs1');

