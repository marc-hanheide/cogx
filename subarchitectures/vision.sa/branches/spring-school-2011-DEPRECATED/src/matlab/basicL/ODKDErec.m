function pcx=ODKDErec(F,mC)

%MINCONF = 3; %minimum number of previously observed objects of the particular category

numSC=getc(mC,'numSC');

for sc=1:numSC
   rslt = executeOperatorIKDEClsfr( mC{sc}, 'input_data', F, 'classifyData', 'use_unknown_model',1   ) ;         
   pcx{sc}=[[getc(mC,sc,0,'name')';0] rslt.P];
   
   
   %rslt = executeOperatorIKDEClsfr( mC{sc}, 'input_data', F, 'calculate_gains'  )  ; 
   
%    [a, dec] = max(rslt.P) ;   
%    disp(['If someone asked me whether the thig was ', num2str(dec),', I would say: ']) ;
%    make_A_decision( dec,  rslt.P ) ;
end

