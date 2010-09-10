function mC=MKDEbatch(F,C)

numC=size(C,1);
namesC=1:numC;
nsbf=1000;

%ESTIMATE KDEs
%select the best F for each C and save the model for each C
mC=struct('name', [], 'kde', [], 'belFun', [], 'conf', []);

for i=1:numC
   mC(i).name=namesC(i);
   idxs=find(C(i,:)==1);
   Fi=F(:,idxs);
   %construct KDE from data
   mC(i).kde= executeOperatorIKDE( [], 'input_data', Fi, 'add_input' );
   mC(i).kde= executeOperatorIKDE( mC(i).kde, 'compress_pdf' );
   %mC(i).belFun=calcBelFun(mC(i).kde.pdf,nsbf);
   mC(i).conf=size(Fi,2);
end;


