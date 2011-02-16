%test3b for running multiple experiments of multiple LM with test3

%use cmlSim to set parameters

%save parameter values
ttloadData=loadData;
ttshowRes=showRes;
ttresEER=resEER;
tLMs=LM;


%load and prepare training and test data
getData;


numExp=3;

numLMs=length(LM);



%inicialize data for displaying learning in progress
testNs=1:5:N;%16;

%Initialize result matrix
%1  1  3  4   5   6  7  8   9  10  11
%Tn,RS,RR,TPF,TNF,NG,NQ,TNQ,RT,NEC,NLC
TTRES=ones(11,length(testNs),numLMs,numExp)*NaN;


if showRes
   figRes=dfigure(6,1,'Evolution of results');
   resizeFigs(figRes,6,1);
   set(figRes,'DefaultAxesLineStyleOrder',{'.-','.:','.--'});
   C=[0 0 1; 0 0.5 0; 1 0 0; 0 0.75 0.75; 0.75 0 0.75; 0.75 0.75 0;
      0.25 0.25 0.25; 0 1 0; 0.5 0 0; 0 0 0.5];
   set(figRes,'DefaultAxesColorOrder',C);
   
end
tnumq=0;



iExp=0;
for iExp=1:numExp
   
   if showRes
      dwaitbar(iExp/numExp,['Running ' num2str(numExp) ' runs...']);
   end;
   
   
   randAll=1;
   loadData=1;
   getData;
   loadData=0;
   showRes=0;
   test3a;
   TTRES(:,:,:,iExp)=TRES;
   MTTRES=nanmean(TTRES,4);
   showRes=ttshowRes;
   
   %plot current state
   if showRes
      
      figure(figRes);
      subplot(1,4,1);
      plot(squeeze(MTTRES(1,:,:)),squeeze(MTTRES(2,:,:)));
      title('RS');
      
      subplot(1,4,2);
      plot(squeeze(MTTRES(1,:,:)),squeeze(MTTRES(3,:,:)));
      title('RR');
      
      subplot(1,4,3);
      plot(squeeze(MTTRES(1,:,:)),squeeze(MTTRES(6,:,:)));
      title('NG');
      
      subplot(1,4,4);
      plot(squeeze(MTTRES(1,:,:)),squeeze(MTTRES(11,:,:))); hold on;
      plot(squeeze(MTTRES(1,:,:)),squeeze(MTTRES(10,:,:))); hold off;
      title('NC');
      
      
   end
   drawnow;
   
end;


%restore parameter values
loadData=ttloadData;
showRes=ttshowRes;
resEER=ttresEER;
LM=tLMs;


