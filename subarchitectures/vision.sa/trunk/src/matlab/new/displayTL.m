function displayTL(mC)

global Coma Disp Settings

if Settings.Disp.TL
   fid=fopen(Disp.mTL,'w');
   
   
   numSC=getc(mC,'numSC');
   fprintf(fid,'numSC: %d,   numComp: %2.1f,    conf: %d<BR>\n',getc(mC,'numSC'),getc(mC,'numComp'),getc(mC,'conf'));
   fprintf(fid,'<BR>\n');
   for sc=1:numSC
      fprintf(fid,'<u>%s:</u> <BR>\n',Coma.SCnames(sc,:));
      fprintf(fid,'SC%d ( numC: %d,   numComp: %2.1f,    conf: %d )<BR>\n',sc,getc(mC,sc,'numC'),getc(mC,sc,'numComp'),getc(mC,sc,'conf'));
      fprintf(fid,'concepts: '); fprintf(fid,'%d ',getc(mC,sc,0,'name')); fprintf(fid,'<BR>\n');
      fprintf(fid,'numComp : '); fprintf(fid,'%2.1f ',getc(mC,sc,0,'numComp')); fprintf(fid,'<BR>\n');
      fprintf(fid,'conf    : '); fprintf(fid,'%d ',getc(mC,sc,0,'conf')); fprintf(fid,'<BR>\n');
      fprintf(fid,'Fb      : '); fprintf(fid,'%d ',getc(mC,sc,0,'Fb')); fprintf(fid,'<BR>\n');
      fprintf(fid,'<BR>\n');
   end
   
   fclose(fid);
   
end