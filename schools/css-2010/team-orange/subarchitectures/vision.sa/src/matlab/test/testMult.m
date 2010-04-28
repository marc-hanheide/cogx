%testMult
%multiple experiments to test incremental learing of concepts


global loadData showRes N0ts NOex THRs
showRes=0;

TThrs=[1.6 2 3]%;1.6 1.8 3;1.6 2 4];%;2 4 5;1.75 2 2.25;1 3 5];
TN0=[1];

NUMEXP=10


%read all data
% getAllData;
% filterData;
%load data;


%test
numRuns=NUMEXP*size(TThrs,1)*size(TN0,1);
for ii=1:NUMEXP
  disp('***************************************************');
  ii
  loadData=1;
  pp=0;
  clear T;
  for jj=1:size(TThrs,1)
    for kk=1:size(TN0,1)
      pp=pp+1
      dwaitbar(((ii-1)*numRuns/NUMEXP+pp)/numRuns,'Running experiments...');
      THRs=TThrs(jj,:);
      N0=TN0(kk);
      N0ts=1;%N0;
      N0ex=10;%N0;
      testIncr;
      T(pp).THRs=THRs;
      T(pp).N0=N0;
      T(pp).RS=RS;
      T(pp).NQ=NQ;
      T(pp).TNQ=TNQ;
      T(pp).NLC=NLC;
      T(pp).NEC=NEC;
      T(pp).Tn=Tn;
      loadData=0;
    end
  end
  save(['res' num2str(ii,'%02d')], 'T')
end



%% Display all results

THRN=[reshape([T.THRs],3,numel([T.THRs])/3);[T.N0]];

for jj=1:size(THRN,2);

   % load computed data
   clear ATHRs AN0 ARS ANQ ATNQ ANLC ANEC ATn
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

   fprintf('%2d: THRs= %4.2f %4.2f %4.2f, N0=%2d: RSs= %5.2f %5.2f %5.2f %5.2f %5.2f\n',...
      jj, THRs(1), THRs(2), THRs(3), N0, RS(1, end), RS(2, end), RS(3, end), RS(4, end), RS(5, end));

end


return;

%% Show results for selected set of parameters (THRs, N0)

jj=3

maxRS=1100;
maxNQ=6;

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
RT=1%calcRT(RS,TNQ);

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
legend('TD','TSc','TSl','EXc','EXl');
xlabel('no of added images');
ylabel('recognition score');

figure(figNQ);
legend('TD','TSc','TSl','EXc','EXl');
xlabel('no of added images');
ylabel('number of questions');

figure(figRT);
legend('TD','TSc','TSl','EXc','EXl');
xlabel('no of added images');
ylabel('rationality score');

figure(figNC);
legend('TD','TSc','TSl','EXc','EXl');
xlabel('no of added images');
ylabel('number of learned C');
%%






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

jj=3

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

  


