function answ=VMinterface(req)

disp(['VMinterface: I will process [' num2str(req) ']!']);

global currState;

[pl1,avw1]=VMprocReq(req);

if pl1(1)==0
   answ=0; %do nothing
else

   if pl1(1)==1
      f=currState.f;
   else
      [x,b,pt3d]=OSinterface;
      f=FEinterface(x,b,pt3d);
      currState.f=f;
      currState.lastRansw=[];
   end;

   ans1=LRinterface(pl1(2),avw1,f);

   [pl2,avw2]=VMprocAns(req,ans1);

   if pl2(1)==0 %do not reply
      answ=0;
   elseif pl2(1)<0 %only reply to tutor
      answ=pl2;
   elseif pl2(1)==1 %update only
      ans2=LRinterface(pl2(2),avw2,f);
      [pl3,avw3]=VMprocAns(req,ans2);
      answ=pl3;
   elseif pl2(1)==2 %update and ask
      ans2=LRinterface(pl2(2),avw2,f);
      answ=pl2(3:end);      
   end;
   currState.lastRansw=answ;
end;

if answ(1)~=0
   DSinterfaceIn(answ);
end;

