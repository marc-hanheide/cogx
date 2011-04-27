function [gain,mC]=ODKDEintrospect(mC)


numSC=getc(mC,'numSC');

for sc=1:numSC
   rslt = executeOperatorIKDEClsfr( mC{sc}, 'introspect' ) ;
   gain{sc}=[getc(mC,sc,0,'name')' rslt.Conf_array'];
   mC{sc}.class_gains=rslt.Conf_array;
end

