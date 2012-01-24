%test3 from test2, broken to more modules, nicer

%use cmlSim to set parameters


%load and prepare training and test data
getData;


%inicialize data for displaying learning in progress
testNs=1:5:N;%16;

%Initialize result matrix
%1  1  3  4   5   6  7  8   9  10  11
%Tn,RS,RR,TPF,TNF,NG,NQ,TNQ,RT,NEC,NLC
RES=zeros(11,length(testNs));

j=0;

if showRes
   figRes=dfigure(6,1,'Evolution of results');
   resizeFigs(figRes,6,1);
   figRes2=dfigure(6,2,'Current results');
   resizeFigs(figRes2,6,1);
end
tnumq=0;


%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%

[mC,mCG,mFS]=MTDinit;


%%%%%%%%%%%%%%%% INCREMENTAL LEARNING %%%%%%%%%%%%%%

for i= 1:N

   if showRes
      dwaitbar(i/N,'Incremental learning...');
   end;
   if pauseNow
      disp('Paused!');drawnow;
      while pauseNow
         pause(.1);
      end
      disp('Resumed!');
   end;

   
   %%%%%% U P D A T E %%%%%%
   %get new feature
   f=Ftr(:,i);
   cgt=Ctrsf(:,i);
   cgt=cgt((cgt~=0));
   
   %update according to selected learning mode
   warning('off','optim:fmincon:SwitchingToMediumScale');
   [mC,mCG,mFS,numq]=LMupdate(LM,f,cgt,mC,mCG,mFS);
   
   tnumq=tnumq+numq;
   [nlc,nec]=numConcepts(mC);
   

   %%%%%% T E S T  %%%%%%

   if ismember(i,testNs) %test and show progress

      %get results
      rCqnt=MTDrec(Ft,mC,mFS);
      rC=qnt2ql(rCqnt,THRst);
      rC=lf2lof(rC,numC);
      resC=evalRes(rC,Ct);
      rt=calcRT(resC.rs,tnumq);
      ng=MTDnumComp(mC);

      %pack current results
      j=j+1;
      RES(:,j)=[i;resC.rs;resC.rr1;resC.tpf1;resC.tnf1;ng;numq;tnumq;rt;nec;nlc];

      %plot current state
      if showRes
         
         figure(figRes);
         subplot(1,4,1);
         plot(RES(1,1:j),RES(2,1:j),'.g-');
         title('RS');

         subplot(1,4,2);
         plot(RES(1,1:j),RES(3,1:j),'.g-');
         title('RR');

         subplot(1,4,3);
         plot(RES(1,1:j),RES(6,1:j),'.g-');
         title('NG');

         subplot(1,4,4);
         plot(RES(1,1:j),RES(11,1:j),'.g-');hold on;
         plot(RES(1,1:j),RES(10,1:j),'.g-');
         title('NC');
         
         figure(figRes2);
         plotRes1(Ct,rC,resC,Cnames,[LMname ': ']);
                  
      end

   end;

end



%print the final results
if showRes
   printRes(resC,['C',lower(LMname),':  '],{'tpf1s','tnf1s'});
   disp(['*** ',LMname,' ***']);printCG(mCG,Fnames,Cnames);
   %dfigure('Models');showLmodels(mC,Fnames,Cnames);
end


if resEER %calulate results at Equal Error Rate

   disp('*** Results at EER ***');

   %   [eer,thr]=findEER(mC,mFS,Ft,Ct);
   [eer,thr]=findEER(mC,mFS,Ft,Ct,[0 3]);
   disp(['THR at EER: ' num2str(thr)]);

   rCqnt=MTDrec(Ft,mC,mFS);
   rC=qnt2ql(rCqnt,[1 1 1]*thr);
   rC=lf2lof(rC,numC);
   resC=evalRes(rC,Ct);

   printRes(resC,'C:  ',{'tpf1s','tnf1s'});

   figRes3=dfigure(6,3,'Results at EER');
   resizeFigs(figRes3,6,1);
   plotRes1(Ct,rC,resC,Cnames,[LMname ': ']);
end
