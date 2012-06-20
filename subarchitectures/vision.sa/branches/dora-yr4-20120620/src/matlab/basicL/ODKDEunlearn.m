function mC=ODKDEunlearn(F,C,mC)

numSC=getc(mC,'numSC');

for sc=1:numSC
   if C(sc)>0
      indat={};
      indat{1}.data=F;
      indat{1}.class_name=num2str(C(sc));
      mC{sc}=executeOperatorIKDEClsfr( mC{sc}, 'input_data', indat, 'unlearn_with_input'  ) ;
      %mC{sc}=executeOperatorIKDEClsfr( mC{sc}, 'input_data', indat, 'add_input' ) ;
      %mC{sc}=executeOperatorIKDEClsfr( mC{sc},'compress_pdf');
      mC{sc}=executeOperatorIKDEClsfr( mC{sc}, 'make_simple_feature_selection') ;
   end
end
