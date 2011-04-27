function [eer,thr]=findEER(mC,mFS,Ft,Ct,thrLims,numC,accTol)

if nargin<5 
   thrLims=[0,1];
end
if nargin<6
   numC=length(mC);
end;   
if nargin<7
   accTol=2;
end;   

rtn=0;

lLim=thrLims(1);
uLim=thrLims(2);
thr=(lLim+uLim)/2;

while rtn==0
   
      rC=qnt2ql(MTDrec(Ft,mC,mFS),[1 1 1]*thr);
      rCtd=lf2lof(rC,numC);
      resCtd=evalRes(rCtd,Ct);

      if abs(resCtd.tpf1-resCtd.tnf1)<10^-accTol
%      if round(resCtd.tpf1*10^accTol)==round(resCtd.tnf1*10^accTol)
         rtn=1;
      elseif resCtd.tpf1<resCtd.tnf1
         lLim=thr;
         thr=(thr+uLim)/2;
      else
         uLim=thr;
         thr=(lLim+thr)/2;
      end;
               
end;

eer=(resCtd.tpf1+resCtd.tnf1)/2;
   
   