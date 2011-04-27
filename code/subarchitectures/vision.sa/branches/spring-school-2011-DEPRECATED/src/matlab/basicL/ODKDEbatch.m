function mC=ODKDEbatch(F,C)

costThreshold.thReconstructive = 0.01 ;         % thresholds on reconstructive and discriminative compression
costThreshold.thDiscriminative =  0.05 ;

mC=ODKDEinit;

numSC=length(C);

for sc=1:numSC
   numC=size(C{sc},1);
   input_data={};
   for c=1:numC
      idxs=find(C{sc}(c,:)==1);
      indat=[] ;
      indat.data=F(:,idxs);
      indat.class=c;
      input_data=horzcat(input_data,indat);
   end;
   
   mC{sc}=executeOperatorIKDEClsfr( mC{sc}, 'input_data', input_data, 'add_input' ) ;
   th_ref=mC{sc}.compressionClusterThresh;   
   mC{sc}=executeOperatorIKDEClsfr( mC{sc},'compress_pdf',  'compressionClusterThresh', costThreshold);
   mC{sc}=executeOperatorIKDEClsfr( mC{sc}, 'compressionClusterThresh', th_ref);
   mC{sc}=executeOperatorIKDEClsfr( mC{sc}, 'make_simple_feature_selection') ;
   
end

