function displayTR(ansYes,ansPy,avu,g)

global Coma Disp Settings

if Settings.Disp.TR
   fid=fopen(Disp.mTR,'w');

   recYes=idx2name(ansYes,Coma.Cnames);
   recPy=idx2name(ansPy,Coma.Cnames);
   
   fprintf(fid,'Recognized: <b>%s</b><BR>\n',recYes);
   fprintf(fid,'Probably  : %s<BR>\n',recPy);
   fprintf(fid,'<BR>\n');
   
   numCS=length(avu);
   
   for sc=1:numCS
      numC=size(avu{sc},1)-1;
      cs=Coma.Cnames(avu{sc}(1:numC,1),:);
      cs1=[cs repmat('    ',numC,1)];
      cs2=reshape(cs1',1,numel(cs1));
      aps=avu{sc}(:,2);
      gain=g{sc}(:,2);
      
      fprintf(fid,'<u>%s:</u> <BR>\n',Coma.SCnames(sc,:));
      fprintf(fid,'    C:  ');
      fprintf(fid,' %sUK',cs2);
      fprintf(fid,'<BR>\n');
      fprintf(fid,'   AP:  ');
      fprintf(fid,'%4.2f  ',aps);
      fprintf(fid,'<BR>\n');
      fprintf(fid,' Gain:  ');
      fprintf(fid,'%4.2f  ',gain);
      fprintf(fid,'<BR>\n');
      fprintf(fid,'<BR>\n');      
   end
   
   fclose(fid);
   
end