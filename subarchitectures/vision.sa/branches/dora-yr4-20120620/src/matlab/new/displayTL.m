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
      
      
      fprintf(fid, '<table border="1"> <tr>') ;fprintf(fid,'\n');
      fprintf(fid,'<td>concepts: </td>'); fprintf(fid,'<td>%d</td>',getc(mC,sc,0,'name')); %fprintf(fid,'<BR>\n');
      fprintf(fid,'\n<tr>\n') ;
      fprintf(fid,'<td>numComp :</td>'); fprintf(fid,'<td>%d</td>',getc(mC,sc,0,'numComp')); %fprintf(fid,'<BR>\n');
      fprintf(fid,'\n<tr>\n') ;
      fprintf(fid,'<td>conf    :</td>'); fprintf(fid,'<td>%d</td>',getc(mC,sc,0,'conf')); %fprintf(fid,'<BR>\n');
      fprintf(fid,'\n<tr>\n') ;
      fprintf(fid,'<td>gains   :</td>'); fprintf(fid,'<td>%3.2f</td>',getc(mC,sc,0,'gains')); %fprintf(fid,'<BR>\n');
      fprintf(fid,'\n<tr>\n') ;
%      fprintf(fid,'<td>Fb      :</td>'); fprintf(fid,'<td>') ; fprintf(fid,'%d, ',getc(mC,sc,0,'Fb')); fprintf(fid,'</td>') ;%fprintf(fid,'<BR>\n');
      fprintf(fid,'<td>Fb      :</td>'); %fprintf(fid,'<td>') ; 
      fbs=getc(mC,sc,0,'Fb');      
      for nam = 1 : size(fbs,2);
         fprintf(fid, '<td>%s </td>', Coma.Fnames(fbs(nam),:)) ;
      end      
      %fprintf(fid,'%d, ',getc(mC,sc,0,'Fb')); 
      %fprintf(fid,'</td>') ;%fprintf(fid,'<BR>\n');
      fprintf(fid, '</table> ') ;
%       fprintf(fid,'<BR>\n');
   end
   
   fclose(fid);
   
end