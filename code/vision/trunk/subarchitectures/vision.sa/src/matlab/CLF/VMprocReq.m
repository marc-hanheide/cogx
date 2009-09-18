function [pl,avw]=VMprocReq(req)

global currState currMode;

quest=-req(1);
if length(req)>1
   avs=req(2:end);
else
   avs=[];
end;

if currMode.learnMode==1 %no learning
   lrn=0;
else %%learning
   lrn=2;
end;

switch quest
   case 1 %'What do you see?'
      pl=[2,1];
      avw=[];
   case 7 %'This is a ',  ' object.'
      pl=[2,lrn];
      avw=av2avw(avs,currMode.wT);
   case 3 %'It is ', '.'
      pl=[1,lrn];
      avw=av2avw(avs,currMode.wT);
   case 4 %'It is also ', '.'
      pl=[1,lrn];
      avw=av2avw(avs,currMode.wT);
   case 15 %'Yes, that''s correct.'
      avs1=currState.lastPos;
      pl=[1,lrn];
      avw=av2avw(avs1,currMode.wT);
%    case 16 %'No, it is ', '.'
%       pl=[1,lrn];
%       avw=av2avw(avs,currMode.wT);
   case 16 %'This is not a ', ' object.'
      pl=[2,lrn];
      avw=av2avw(avs,-1);
   case 17 %'No, it is not ', '.'
      avs1=currState.lastPos;
      avs2=setdiff(avs1,avs);
      pl=[1,lrn];
      avw=av2avw(avs2,currMode.wT);
      %add negative weights as well
      avw=[av2avw(avs2);av2avw(avs,-1)];
   case 1000 %Attention trigger
      if currMode.learnMode<3 %NL,TD
         pl=[0 0];
         avw=[];
      else %TS,EX
         pl=[2 1];
         avw=[];
      end
end;

VMintProcUpdate(1,pl,avw);



