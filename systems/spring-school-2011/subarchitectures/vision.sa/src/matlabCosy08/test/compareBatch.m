%compareBatch


MTDs=1:6;
THRss=[.6 3 0 0 .6 .6];

numMTDs=length(MTDs);
TR=zeros(numMTDs,3);

for i=1:numMTDs
   MTD=MTDs(i);
   THRs=THRss(i);
   testBatch;
   TR(i,:)=[resCt.rs,resCt.tpf1,resCt.tnf1];
end;


dfigure(2,3,'RS');
bar(TR(:,1));
setAxis([],[],[],resCt.rsmax);

dfigure('TPF,TNF');
bar(TR(:,2:3));
legend('TP','TN',4);

