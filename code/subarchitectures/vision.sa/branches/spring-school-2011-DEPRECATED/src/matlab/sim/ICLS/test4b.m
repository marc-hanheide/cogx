%test4b for running multiple experiments of multiple LM with test4

%use cmlSim to set parameters

%save parameter values
ttloadData=loadData;
ttshowRes=showRes;
ttresEER=resEER;
tLMs=LM;


%load and prepare training and test data
getFCdata;


%numRuns=3;

numLMs=length(LM);


%multKDBF13: 100/100/.2 .05 .01/.06 .05 .02/cthr=5/scaleErrorThreshold = 2 ; hellErrorGlobal = 0.3; %DS!
%multKDBF13a: 100/100/.2 .05 .01/.06 .05 .02/cthr=6/scaleErrorThreshold =  1/0.7 ; hellErrorGlobal = 0.1; %DS!


%inicialize data for displaying learning in progress
%testNs=1:5:N;%16;
testNs=[1 5:5:N];

%Initialize result matrix
%1  1  3  4   5   6  7  8   9  10  11
%Tn,RS,RR,TPF,TNF,NG,NQ,TNQ,RT,NEC,NLC
TTRES=ones(11,length(testNs),numLMs,numRuns)*NaN;


if showRes
   figRes=dfigure(6,1,'Evolution of results');
   resizeFigs(figRes,6,1);
   set(figRes,'DefaultAxesLineStyleOrder',{'.-','.:','.--'});
   cmap=[0 0 1; 0 0.5 0; 1 0 0; 0 0.75 0.75; 0.75 0 0.75; 0.75 0.75 0;
      0.25 0.25 0.25; 0 1 0; 0.5 0 0; 0 0 0.5];
   set(figRes,'DefaultAxesColorOrder',cmap);
   
end
tnumq=0;

showProgress(3,0);


iExp=0;
for iExp=1:numRuns
   
%    if showRes
%       dwaitbar(iExp/numRuns,['Running ' num2str(numRuns) ' runs...']);
%    end;
   
   
   randAll=1;
   loadData=1;
   getFCdata;
   loadData=0;
   showRes=0;
   test4a;
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
      plot(squeeze(MTTRES(1,:,:)),squeeze(MTTRES(7,:,:))); 
      title('NQ');
      
      
   end
   drawnow;
   showProgress(3,iExp/numRuns);

end;


%restore parameter values
loadData=ttloadData;
showRes=ttshowRes;
resEER=ttresEER;
LM=tLMs;


disp('Test completed.');

save('expEpirob2');