%test4 from test3, further cosmetic modifications
%use cmlSim to set parameters

%load and prepare training and test data using cmlSim or
%getFCdata;


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

showProgress(1,0);


%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%%

%[mC,mCG,mFS]=MTDinit;
%mC=ODKDEinit;

Ctrsf=lnf2snf(Ctr);

%%%%%%%%%%%%%%%% INCREMENTAL LEARNING %%%%%%%%%%%%%%

for i= 1:N

   if showRes
      %dwaitbar(i/N,'Incremental learning...');
   end;
   if pauseNow
      disp('Paused!');drawnow;
      while pauseNow
         pause(.1);
      end
      disp('Resumed!');
   end;
   if stopNow
      disp('Teriminating ICLS...');
      error('Teriminating ICLS...');
   end;

   
   %%%%%% U P D A T E %%%%%%
   %get new feature
   f=Ftr(:,i);
   cgt=Ctrsf(:,i);
   %cgt=cgt((cgt~=0));
   
   %update according to selected learning mode
   
%    if i<=Nini
%       [mC,mCG,mFS,numq]=LMupdate(1,f,cgt,mC,mCG,mFS);
%    else
%       [mC,mCG,mFS,numq]=LMupdate(LM,f,cgt,mC,mCG,mFS);
%    end
   
   mC=ODKDEupdate(f,cgt,mC)
   
%   tnumq=tnumq+numq;
%   [nlc,nec]=numConcepts(mC);
   tnumq=0;
   nlc=0;nec=0;

   %%%%%% T E S T  %%%%%%

   if ismember(i,testNs) %test and show progress

      
      %get results
      rCCpcx=ODKDErec(Ft,mC);
rCpcx=cc2c(rCCpcx,'trim');
rC=qnt2ql(rCpcx,THRst);

%      rCqnt=MTDrec(Ft,mC,mFS);
%      rC=qnt2ql(rCqnt,THRst);
%      rC=lf2lof(rC,numC);
      resC=evalRes(rC,Ct);
      rt=calcRT(resC.rs,tnumq);
      ng=MTDnumComp(mC);

      %pack current results
      j=j+1;
      RES(:,j)=[i;resC.rs;resC.rr1;resC.tpf1;resC.tnf1;ng;tnumq;tnumq;rt;nec;nlc];

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
         plot(RES(1,1:j),RES(7,1:j),'.g-');
         title('NQ');
         
         figure(figRes2);
         plotRes1(Ct,rC,resC,Cnames,[LMname ': ']);   
         
      end

      showProgress(1,i/N);

   end;

   
end



%print the final results
if showRes
   printRes(resC,['C',lower(LMname),':  '],{'tpf1s','tnf1s'});
   %disp(['*** ',LMname,' ***']);printCG(mCG,Fnames,Cnames);
   MTDdispModel(mC,mCG,Fnames,Cnames);
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

disp('Finished 1ML1!');
