%test4a for running multiple LM with test4

%use cmlSim to set parameters

%save parameter values
tloadData=loadData;
tshowRes=showRes;
tresEER=resEER;
LMs=LM;


%load and prepare training and test data
%getFCdata;

%sortTrData;

numLMs=length(LM);


%inicialize data for displaying learning in progress
%testNs=1:5:N;%16;
testNs=[1 5:5:N];

%Initialize result matrix
%1  1  3  4   5   6  7  8   9  10  11
%Tn,RS,RR,TPF,TNF,NG,NQ,TNQ,RT,NEC,NLC
TRES=zeros(11,length(testNs),numLMs);


if showRes
   figRes=dfigure(6,1,'Evolution of results');
   resizeFigs(figRes,6,1);
   set(figRes,'DefaultAxesLineStyleOrder',{'.-','.:','.--'});
   cmap=[0 0 1; 0 0.5 0; 1 0 0; 0 0.75 0.75; 0.75 0 0.75; 0.75 0.75 0;
      0.25 0.25 0.25; 0 1 0; 0.5 0 0; 0 0 0.5];
   set(figRes,'DefaultAxesColorOrder',cmap);
   drawnow;
end
tnumq=0;

% loadData=1;
% getFCdata;
 loadData=0;


showProgress(2,0);

iLm=0;
for iLm=1:numLMs
   LM=LMs(iLm);
   showRes=0;
   test4;
   TRES(:,:,iLm)=RES;
   showRes=tshowRes;
   
   %plot current state
   if showRes
      figure(figRes);
      subplot(1,4,1);
      plot(squeeze(TRES(1,1:j,:)),squeeze(TRES(2,1:j,:)));
      title('RS');
      
      subplot(1,4,2);
      plot(squeeze(TRES(1,1:j,:)),squeeze(TRES(3,1:j,:)));
      title('RR');
      
      subplot(1,4,3);
      plot(squeeze(TRES(1,1:j,:)),squeeze(TRES(6,1:j,:)));
      title('NG');
      
      subplot(1,4,4);
      plot(squeeze(TRES(1,1:j,:)),squeeze(TRES(7,1:j,:)));
      title('NQ');
      
      drawnow;
      
   end
   
   showProgress(2,iLm/numLMs);

end;


%restore parameter values
loadData=tloadData;
showRes=tshowRes;
resEER=tresEER;
LM=LMs;


