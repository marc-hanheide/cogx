%test4 from test3, further cosmetic modifications
%use cmlSim to set parameters

%load and prepare training and test data using cmlSim or
%getFCdata;


%Initialize result matrix
%1  1  3  4   5   6  7  8   9  10  11
%Tn,RS,RR,TPF,TNF,NG,NQ,TNQ,RT,NEC,NLC
%Tn,RS,RR,TPF,TNF,unknM,weakM,weakR,ambM,NEC,NLC
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

[mC,mCG,mFS]=MTDinit;


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
   cgt=cgt((cgt~=0));
   
   %update according to selected learning mode
   
   if i<=Nini
      [mC,mCG,mFS,numq]=LMupdate(1,f,cgt,mC,mCG,mFS);
   else
      [mC,mCG,mFS,numq]=LMupdate(LM,f,cgt,mC,mCG,mFS);
   end
   
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

      g=detectGaps(rCqnt,mC);
      
      %pack current results
      j=j+1;
      RES(:,j)=[i;resC.rs;resC.rr1;resC.tpf1;resC.tnf1;g(1);g(2);g(3);g(4);nec;nlc];

      %plot current state
      if showRes
         
         figure(figRes);
         subplot(1,4,1);
         plot(RES(1,1:j),RES(6,1:j),'.g-');
         title('RS');

         subplot(1,4,2);
         plot(RES(1,1:j),RES(7,1:j),'.g-');
         title('RR');

         subplot(1,4,3);
         plot(RES(1,1:j),RES(8,1:j),'.g-');
         title('NEC');

         subplot(1,4,4);
         plot(RES(1,1:j),RES(9,1:j),'.g-');
         title('NLC');
         
         figure(figRes2);
         plotRes1(Ct,rC,resC,Cnames,[LMname ': ']);   
         
      end

      showProgress(1,i/N);

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

disp('Finished 1ML1!');
