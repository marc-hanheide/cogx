function gain=ODKDEgain(F,mC)

%MINCONF = 3; %minimum number of previously observed objects of the particular category

numSC=getc(mC,'numSC');

for sc=1:numSC
   %rslt = executeOperatorIKDEClsfr( mC{sc}, 'input_data', F, 'calculate_gains'  )  ; 
   rslt = executeOperatorIKDEClsfr( mC{sc}, 'input_data', F, 'classifyData', 'use_unknown_model',1   ) ;         
   %ap=[[getc(mC,sc,0,'name')';0] rslt.P];
   %ap=[[getc(mC,sc,0,'name')';0] rslt.P];
   gain{sc}=[getc(mC,sc,0,'name')' ap2gain(rslt.P(1:end-1))];
   %ap2gain(ap);
end

