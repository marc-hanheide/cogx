function [pl,avw]=VMprocAns(req,ansLr)

readConstants;
global currMode currState Params

currState.lastPos=[];
currState.lastNeg=[];
currState.lastYes=[];
currState.lastPy=[];


if size(ansLr,2)==1 %from update
   fromUpd=1;
else
   fromUpd=0;

   rCpcx=cc2c(ansLr,'trim');
   ansQl=qnt2ql(rCpcx,Params.THRs);
      
   ansYes=lf2sfa(ansQl,ANSyes);
   ansPy=lf2sfa(ansQl,ANSpy);
end;

if -req(1)<1000 %tutor initiated conversation

   if fromUpd %from update
      pl=[-12]; %'OK.', ''
      avw=[];
   else
      if ~isempty(ansYes)
         pl=[-8 ansYes]; %'I see a ',  ' object.'
         avw=[];
         currState.lastPos=ansYes;
         if ~isempty(ansPy)
            pl=[pl -11 ansPy]; %'I think it is also '
            avw=[];
            currState.lastPos=[currState.lastPos ansPy];
         end
      elseif ~isempty(ansPy)
         pl=[-14 ansPy]; %'I think I see a ', ' object.'
         avw=[];
         currState.lastPos=ansPy;
      else
         pl=[-9]; %'I don''t know this object.', ''
         avw=[];
      end
      currState.lastYes=ansYes;
      currState.lastPy=ansPy;
   end;

else %robot initiative

   switch currMode.learnMode

      case 3 %TSc

         if fromUpd %from update
            pl=0; %no answer
            avw=[];
         else %from recognise
            if ~isempty(ansYes) && isempty(ansPy) %update only
               pl=[1 2]; %update
               avw=av2avw(ansYes,currMode.wYes);
            elseif isempty(ansYes) && ~isempty(ansPy) %ask only
               pl=[-18 ansPy]; %'Is this object ', '?.'
               avw=[];
               currState.lastPos=ansPy;
            elseif ~isempty(ansYes) && ~isempty(ansPy) %update and ask
               pl=[2 2 -18 ansPy]; %upd and 'Is this object ', '?.'
               avw=av2avw(ansYes,currMode.wYes);
               currState.lastPos=ansPy;
            else
               pl=[-9 -19];
               avw=[];
            end
            currState.lastYes=ansYes;
            currState.lastPy=ansPy;
         end;

      case 4 %TSl

         if fromUpd %from update
            pl=0; %no answer
            avw=[];
         else %from recognise
            if ~isempty(ansYes) || ~isempty(ansPy) %update only
               pl=[1 2]; %update
               avw=[av2avw(ansYes,currMode.wYes);av2avw(ansPy,currMode.wPy)];
            else
               pl=[-9 -19];
               avw=[];
            end
            currState.lastYes=ansYes;
            currState.lastPy=ansPy;
         end;


      case 5 %EXc

         if fromUpd %from update
            pl=0; %no answer
            avw=[];
         else
            if ~isempty(ansYes)
               pl=[1 2]; %update
               avw=av2avw(ansYes,currMode.wYes);
            else
               pl=0;
               avw=[];
            end
            currState.lastYes=ansYes;
            currState.lastPy=ansPy;
         end;

      case 6 %EXl

         if fromUpd %from update
            pl=0; %no answer
            avw=[];
         else
            if ~isempty(ansYes) || ~isempty(ansPy)
               pl=[1 2]; %update
               avw=[av2avw(ansYes,currMode.wYes);av2avw(ansPy,currMode.wPy)];
            else
               pl=0;
               avw=[];
            end
            currState.lastYes=ansYes;
            currState.lastPy=ansPy;
         end;


   end;
end;

VMintProcUpdate(2,pl,avw,ansLr);
