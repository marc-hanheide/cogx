%test3c for running multiple experiments of multiple LM with several THRS with test3

%use cmlSim to set parameters

%save parameter values
ttloadData=loadData;
ttshowRes=showRes;
ttresEER=resEER;
tLMs=LM;


%load and prepare training and test data
getData;


numExp=2;
TTHRs=[.2 .1 .05;
       .1 .02 .01;
       .06 .05 .01;
       .05 .04 .002;
       .08 .05 .01;
       .06 .05 .03;
       .04 .03 .01;
       .06 .05 .001;
       .1 .05 .01;
       .15 .05 .01];

TTHRs=[.1 .02 .01;
       .06 .05 .01;
       .05 .04 .01;
       ];
    
numLMs=length(LM);
numTHRs=size(TTHRs,1);


%inicialize data for displaying learning in progress
testNs=1:5:N;%16;

%Initialize result matrix
%1  1  3  4   5   6  7  8   9  10  11
%Tn,RS,RR,TPF,TNF,NG,NQ,TNQ,RT,NEC,NLC
TTRES=ones(11,length(testNs),numLMs,numTHRs,numExp)*NaN;


if showRes
   figRes=dfigure(6,1,'Evolution of results');
   resizeFigs(figRes,6,1);
   set(figRes,'DefaultAxesLineStyleOrder',{'.-','.:','.--'});
   C=[0 0 1; 0 0.5 0; 1 0 0; 0 0.75 0.75; 0.75 0 0.75; 0.75 0.75 0;
      0.25 0.25 0.25; 0 1 0; 0.5 0 0; 0 0 0.5];
   set(figRes,'DefaultAxesColorOrder',C);
   
end
tnumq=0;



for iExp=1:numExp
   
   disp(['******** Run ' num2str(iExp) '/' num2str(numExp) ' ********']);
   
   if showRes
      dwaitbar(iExp/numExp,['Running ' num2str(numExp) ' runs...']);
   end;
   
   
   randAll=1;
   loadData=1;
   getData;
   loadData=0;
   showRes=0;
   
   for iTHRs=1:numTHRs
      THRs=TTHRs(iTHRs,:);
      test3a;
      TTRES(:,:,:,iTHRs,iExp)=TRES;
   end;
   
   TMTTRES=nanmean(TTRES,5);
   [bestRS,bestTHRs]=max(TMTTRES(2,end,1,:));
   MTTRES=TMTTRES(:,:,:,bestTHRs);
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

      drawnow;
      
   end
   
end;


%restore parameter values
loadData=ttloadData;
showRes=ttshowRes;
resEER=ttresEER;
LM=tLMs;

disp('Test completed.');


