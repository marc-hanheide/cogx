%test4c for running multiple experiments of multiple LM with several THRS
%with test4

%use cmlSim to set parameters

%save parameter values
ttloadData=loadData;
ttshowRes=showRes;
ttresEER=resEER;
tLMs=LM;

%multKDBF4, THRSt=THRs (also multKDBF5)
%numRuns=5;
TTHRs=[.2 .1 .05;
       .1 .02 .01;
       .06 .05 .01;
       .05 .04 .002;
       .08 .05 .01;
       .06 .05 .03;
       .04 .03 .01;
       .06 .05 .001;
       .1 .05 .01;
       .15 .05 .01;
       .02 .002 .001];
 THRst=[.06 .05 .02]; %multKDBF6, MINCONF=2 up to now
 
 %multKDBF7, MINCONF=6 
 %multKDBF9: Niti=20, MINCONF=2
% 
%  
% %numRuns=5;
% TTHRs=[.2 .1 .05; %multKDBF10, MINCONF=2 
%        .1 .02 .01;
%        .06 .05 .01;
%        .5 .1 .0001;
%        .9 .05 .0000001;
%        1 .1 0;
% ];
%  THRst=[.06 .05 .02]; 
% 
% 
% %numRuns=10;
% TTHRs=[.2 .1 .05; %multKDBF10, MINCONF=2 
%        .1 .02 .01;
%        .25 .05 .02;
%        .25 .05 .005;
%        .5 .05 .005;
% ];
%  THRst=[.06 .05 .02]; 
%  
% %numRuns=5;
% TTHRs=[.2 .1 .05; %multKDBF12, MINCONF=5 
%        .25 .05 .02;
% ];
%  THRst=[.06 .05 .02]; 
%  
% 
% % TTHRs=[.1 .02 .01;
% %        .06 .05 .01;
% %        .05 .04 .01;
% %        ];
% %     



 
 
numLMs=length(LM);
numTHRs=size(TTHRs,1);


%inicialize data for displaying learning in progress
%testNs=1:5:N;%16;
testNs=[1 5:5:N];

%Initialize result matrix
%1  1  3  4   5   6  7  8   9  10  11
%Tn,RS,RR,TPF,TNF,NG,NQ,TNQ,RT,NEC,NLC
TTRES=ones(11,length(testNs),numLMs,numTHRs,numRuns)*NaN;


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

for iExp=1:numRuns
   
   disp(['******** Run ' num2str(iExp) '/' num2str(numRuns) ' ********']);
   
%    if showRes
%       dwaitbar(iExp/numRuns,['Running ' num2str(numRuns) ' runs...']);
%    end;
   
   
   randAll=1;
   loadData=1;
   getFCdata;
   loadData=0;
   showRes=0;
   showProgress(4,0);

   for iTHRs=1:numTHRs
      THRs=TTHRs(iTHRs,:);
      %THRst=THRs;
      test4a;
      TTRES(:,:,:,iTHRs,iExp)=TRES;
      showProgress(4,iTHRs/numTHRs);
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
      plot(squeeze(MTTRES(1,:,:)),squeeze(MTTRES(7,:,:))); 
      title('NQ');

      drawnow;
      
   end
   
      showProgress(3,iExp/numRuns);
   
end;


%restore parameter values
loadData=ttloadData;
showRes=ttshowRes;
resEER=ttresEER;
LM=tLMs;

disp('Test completed.');

return


%%
iTHRs=3;

TTHRs(iTHRs,:)

   MTTRES=TMTTRES(:,:,:,iTHRs);
   showRes=1;
   
   %plot current state
   if showRes
      
         figRes=dfigure(6,1,'Evolution of results');
   resizeFigs(figRes,6,1);
   set(figRes,'DefaultAxesLineStyleOrder',{'.-','.:','.--'});
   C=[0 0 1; 0 0.5 0; 1 0 0; 0 0.75 0.75; 0.75 0 0.75; 0.75 0.75 0;
      0.25 0.25 0.25; 0 1 0; 0.5 0 0; 0 0 0.5];
   set(figRes,'DefaultAxesColorOrder',C);

      
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

      drawnow;
      
   end


   return;

   %% BW RS
%   set(figRes,'DefaultAxesLineStyleOrder',{'o-','.-','.--','x-','x--'});
   set(figRes,'DefaultAxesLineStyleOrder','.-');
%   C=[0 0 0; .25 .25 .55;  .25 .25 .55; .5 .5 .5;  .5 .5 .5;];
   C=[0 0 0; 0 0 1;0 0 1;1 0 0;1 0 0];
   set(figRes,'DefaultAxesColorOrder',C);
   
         plot(squeeze(MTTRES(1,:,:)),squeeze(MTTRES(2,:,:)));
      title('RS');

   legend('TD','TSc','TSl','EXc','EXl');
   xlabel('no. of added images');
   ylabel('recognition score');
   axis([0 100 0 1000]);

   ylabel('number of questions');
