%testMult
%multiple experiments to test incremental learing of concepts


global loadData showRes THRs
showRes=0;

TThrs=[.90 .92 .93 .95 .98]%:.01:.87%[.85 .90 .92 .95 .97];%[1.4 1.6 1.8 2]
%TThrs=2%[1.2 1.4 1.6 1.8 2];MTD=4;
TThrs=([1;1;1]*TThrs)';
MTD=3

NUMEXP=20


%read all data
% getAllData;
% filterData;
%load data;


%test
numRuns=NUMEXP*size(TThrs,1);
for ii=1:NUMEXP
  disp('***************************************************');
  ii
  loadData=1;
  pp=0;
  clear T;
  for jj=1:size(TThrs,1)
      pp=pp+1
      dwaitbar(((ii-1)*numRuns/NUMEXP+pp)/numRuns,'Running experiments...');
      THRs=TThrs(jj,:);
      testIncrTD;
      T(pp).THRs=THRs;
      T(pp).RS=RS;
      T(pp).NQ=NQ;
      T(pp).TNQ=TNQ;
      T(pp).NLC=NLC;
      T(pp).NEC=NEC;
      T(pp).Tn=Tn;
      T(pp).RR=RR;
      T(pp).TPF=TPF;
      T(pp).TNF=TNF;
      T(pp).NG=NG;
  end
  save(['res' num2str(ii,'%02d')], 'T')
end



%% Display all results

load('res01');
THRN=[reshape([T.THRs],3,numel([T.THRs])/3)];

for jj=1:size(THRN,2);

   % load computed data
   clear ATHRs AN0 ARS ANQ ATNQ ANLC ANEC ATn ARR ATPF ATNF ANG
   for ii=1:NUMEXP
      load(['res' num2str(ii,'%02d')]);
      ATHRs(:,ii)=T(jj).THRs;
      ARS(:,:,ii)=T(jj).RS;
      ANQ(:,:,ii)=T(jj).NQ;
      ATNQ(:,:,ii)=T(jj).TNQ;
      ANLC(:,:,ii)=T(jj).NLC;
      ANEC(:,:,ii)=T(jj).NEC;
      ATn(:,ii)=T(jj).Tn;
      ARR(:,:,ii)=T(jj).RR;
      ATPF(:,:,ii)=T(jj).TPF;
      ATNF(:,:,ii)=T(jj).TNF;
      ANG(:,:,ii)=T(jj).NG;
   end;

   % get means
   THRs=mean(ATHRs,2);
   RS=mean(ARS,3);
   NQ=mean(ANQ,3);
   TNQ=mean(ATNQ,3);
   NLC=mean(ANLC,3);
   NEC=mean(ANEC,3);
   Tn=mean(ATn,2);
   RR=mean(ARR,3);
   TPF=mean(ATPF,3);
   TNF=mean(ATNF,3);
   NG=mean(ANG,3);

   fprintf('%2d: THRs= %4.2f %4.2f %4.2f:  RS= %5.2f  RR=%5.2f  TPF=%5.2f  TNF=%5.2f  NG=%5.2f\n',...
      jj, THRs(1), THRs(2), THRs(3), RS(end), RR(end), TPF(end), TNF(end), NG(end));

end


return;

%% Show results for selected set of parameters (THRs)

jj=4

maxRS=2100;
maxNQ=9;
numC=21;

   % load computed data
   clear ATHRs AN0 ARS ANQ ATNQ ANLC ANEC ATn ARR ATPF ATNF ANG
   for ii=1:NUMEXP
      load(['res' num2str(ii,'%02d')]);
      ATHRs(:,ii)=T(jj).THRs;
      ARS(:,:,ii)=T(jj).RS;
      ANQ(:,:,ii)=T(jj).NQ;
      ATNQ(:,:,ii)=T(jj).TNQ;
      ANLC(:,:,ii)=T(jj).NLC;
      ANEC(:,:,ii)=T(jj).NEC;
      ATn(:,ii)=T(jj).Tn;
      ARR(:,:,ii)=T(jj).RR;
      ATPF(:,:,ii)=T(jj).TPF;
      ATNF(:,:,ii)=T(jj).TNF;
      ANG(:,:,ii)=T(jj).NG;
   end;

   % get means
   THRs=mean(ATHRs,2);
   RS=mean(ARS,3);
   NQ=mean(ANQ,3);
   TNQ=mean(ATNQ,3);
   NLC=mean(ANLC,3);
   NEC=mean(ANEC,3);
   Tn=mean(ATn,2);
   RR=mean(ARR,3);
   TPF=mean(ATPF,3);
   TNF=mean(ATNF,3);
   NG=mean(ANG,3);
   
% plot results
j=size(RS,2);
figRS=dfigure(4,1,'RS');
plot(Tn(1:j),RS(1,1:j),'.b-');hold on;
setaxis(0,max(Tn),0,maxRS);

figNQ=dfigure('NQ');
plot(Tn(1:j),NQ(1,1:j),'.b-');hold on;
setaxis(0,max(Tn),0,maxNQ);

figRT=dfigure('RT');
%plot(Tn(1:j),RT(1,1:j),'.b-');hold on;
setaxis(0,max(Tn),0,[]);

figNC=dfigure('NC');
plot(Tn(1:j),NLC(1,1:j),'.b-');hold on;
plot(Tn(1:j),NEC(1,1:j),'.b-');
setaxis(0,max(Tn),0,numC);

figRR=dfigure('RR');
plot(Tn(1:j),RR(1,1:j),'.b-');hold on;
setaxis(0,max(Tn),0,100);
  
figTPF=dfigure('TPF');
plot(Tn(1:j),TPF(1,1:j),'.b-');hold on;
setaxis(0,max(Tn),0,1);

figTNF=dfigure('TNF');
plot(Tn(1:j),TNF(1,1:j),'.b-');hold on;
setaxis(0,max(Tn),0,1);

figNG=dfigure('NG');
plot(Tn(1:j),NG(1,1:j),'.b-');hold on;
setaxis(0,max(Tn),0,20);





%% plot results in BW

setfonts(16)
maxTn=100;
Nex=5;

j=size(RS,2);
figRS=dfigure(4,1,'RS');
plot(Tn(1:j),RS(1,1:j),'ok-');hold on;
plot(Tn(1:j),RS(2,1:j),'.k--');plot(Tn(1:j),RS(3,1:j),'.k:');
plot(Tn(Nex:j),RS(4,Nex:j),'xk--');plot(Tn(Nex:j),RS(5,Nex:j),'xk:');
setaxis(0,maxTn,0,maxRS);

figNQ=dfigure('NQ');
plot(Tn(1:j),NQ(1,1:j),'ok-');hold on;
plot(Tn(1:j),NQ(2,1:j),'.k--');plot(Tn(1:j),NQ(3,1:j),'.k:');
plot(Tn(Nex:j),NQ(4,Nex:j),'xk--');plot(Tn(Nex:j),NQ(5,Nex:j),'xk:');
setaxis(0,maxTn,0,maxNQ);

figRT=dfigure('RT');
plot(Tn(1:j),RT(1,1:j),'ok-');hold on;
plot(Tn(1:j),RT(2,1:j),'.k--');plot(Tn(1:j),RT(3,1:j),'.k:');
plot(Tn(1:j),RT(4,1:j),'xk--');plot(Tn(Nex:j),RT(5,Nex:j),'xk:');
setaxis(0,maxTn,0,[]);

figNC=dfigure('NC');
plot(Tn(1:j),NLC(1,1:j),'ok-');hold on;
plot(Tn(1:j),NLC(2,1:j),'.k--');plot(Tn(1:j),NLC(3,1:j),'.k:');
plot(Tn(1:j),NLC(4,Nex:j),'xk--');plot(Tn(1:j),NLC(5,Nex:j),'xk:');
plot(Tn(1:j),NEC(1,1:j),'ok-');
plot(Tn(1:j),NEC(2,1:j),'.k--');plot(Tn(1:j),NEC(3,1:j),'.k:');
plot(Tn(1:j),NEC(4,Nex:j),'xk--');plot(Tn(1:j),NEC(5,Nex:j),'xk:');
setaxis(0,maxTn,0,maxNQ);


%% add legends to figures
figure(figRS);
xlabel('no of added images');
ylabel('recognition score');

figure(figNQ);
xlabel('no of added images');
ylabel('number of questions');

figure(figRT);
xlabel('no of added samples');
ylabel('rationality score');

figure(figNC);
xlabel('no of added images');
ylabel('number of learned C');

figure(figRR);
xlabel('no of added samples');
ylabel('accuracy');

figure(figTPF);
xlabel('no of added images');
ylabel('TP fraction');

figure(figTNF);
xlabel('no of added images');
ylabel('TN fraction');

figure(figNG);
xlabel('no of added samples');
ylabel('average number of components');


%% enlarge font

figure(figRR);
set(gca,'FontSize',20);
xlabel('no of added samples');
ylabel('accuracy');

figure(figNG);
set(gca,'FontSize',20);
xlabel('no of added samples');
ylabel('average number of components');



%% add legends to figures SI
figure(figRS);
legend('PU','DNk','DNl','PSk','PSl');
xlabel('št. dodanih slik');
ylabel('uspešnost razpoznavanja');

figure(figNQ);
legend('PU','DNk','DNl','PSk','PSl');
xlabel('št. dodanih slik');
ylabel('število vprašanj');

figure(figRT);
legend('PU','DNk','DNl','PSk','PSl');
xlabel('št. dodanih slik');
ylabel('racionalnost');

figure(figNC);
legend('PU','DNk','DNl','PSk','PSl');
xlabel('št. dodanih slik');
ylabel('št. auèenih konceptov');
%%

















%% Show results for selected set of parameters (THRs, N0)

jj=1

maxRS=1500;
maxNQ=10;

% load computed data
clear ATHRs AN0 ARS ANQ ATn
for ii=1:NUMEXP
  load(['res' num2str(ii,'%02d')]);
  ATHRs(:,ii)=T(jj).THRs;
  AN0(:,ii)=T(jj).N0;
  ARS(:,:,ii)=T(jj).RS;
  ANQ(:,:,ii)=T(jj).NQ;
  ATNQ(:,:,ii)=T(jj).TNQ;
  ANLC(:,:,ii)=T(jj).NLC;
  ANEC(:,:,ii)=T(jj).NEC;
  ATn(:,ii)=T(jj).Tn;
end;

% get means
THRs=mean(ATHRs,2);
N0=mean(AN0,2);
RS=mean(ARS,3);
NQ=mean(ANQ,3);
TNQ=mean(ATNQ,3);
NLC=mean(ANLC,3);
NEC=mean(ANEC,3);
Tn=mean(ATn,2);
RT=calcRT(RS,TNQ);

% plot results
j=size(RS,2);
figRS=dfigure(4,1,'RS');
plot(Tn(1:j),RS(1,1:j),'.g-');hold on;
plot(Tn(1:j),RS(2,1:j),'+b--');plot(Tn(1:j),RS(3,1:j),'+b:');plot(Tn(1:j),RS(4,1:j),'xr--');plot(Tn(1:j),RS(5,1:j),'xr:');
setaxis(0,max(Tn),0,maxRS);

figNQ=dfigure('NQ');
plot(Tn(1:j),NQ(1,1:j),'.g-');hold on;
plot(Tn(1:j),NQ(2,1:j),'+b--');plot(Tn(1:j),NQ(3,1:j),'+b:');plot(Tn(1:j),NQ(4,1:j),'xr--');plot(Tn(1:j),NQ(5,1:j),'xr:');
setaxis(0,max(Tn),0,maxNQ);

figRT=dfigure('RT');
plot(Tn(1:j),RT(1,1:j),'.g-');hold on;
plot(Tn(1:j),RT(2,1:j),'+b--');plot(Tn(1:j),RT(3,1:j),'+b:');plot(Tn(1:j),RT(4,1:j),'xr--');plot(Tn(1:j),RT(5,1:j),'xr:');
setaxis(0,max(Tn),0,[]);

figNC=dfigure('NC');
plot(Tn(1:j),NLC(1,1:j),'.g-');hold on;
plot(Tn(1:j),NLC(2,1:j),'+b--');plot(Tn(1:j),NLC(3,1:j),'+b:');plot(Tn(1:j),NLC(4,1:j),'xr--');plot(Tn(1:j),NLC(5,1:j),'xr:');
plot(Tn(1:j),NEC(1,1:j),'.g-');
plot(Tn(1:j),NEC(2,1:j),'+b--');plot(Tn(1:j),NEC(3,1:j),'+b:');plot(Tn(1:j),NEC(4,1:j),'xr--');plot(Tn(1:j),NEC(5,1:j),'xr:');
setaxis(0,max(Tn),0,maxNQ);

  


