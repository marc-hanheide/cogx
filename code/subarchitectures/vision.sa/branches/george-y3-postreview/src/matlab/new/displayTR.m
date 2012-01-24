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
      if avu{sc}(1)~=0
         numC=size(avu{sc},1)-1;
         cs=Coma.Cnames(avu{sc}(1:numC,1),:);
         cs1=[cs repmat('    ',numC,1)];
         cs2=reshape(cs1',1,numel(cs1));
         aps=avu{sc}(:,2);
         gain=g{sc}(:,2);
         
         fprintf(fid,'<u>%s:</u> <BR>\n',Coma.SCnames(sc,:));
         fprintf(fid, '<table border="1"> <tr>') ;fprintf(fid,'\n');
         fprintf(fid,'<td>') ; fprintf(fid,'    C:  '); fprintf(fid,'</td>') ; fprintf(fid,'\n');
         for nam = 1 : size(cs1,1)
             fprintf(fid, '<td>%s</td>', cs1(nam,:)) ;
         end
         fprintf(fid,'<td>UK</td>'); fprintf(fid, '\n <tr> \n') ; 
%          fprintf(fid,'<td>%s</td><td>UK</td>',cs2); fprintf('\n<tr>\n') ; 
%          fprintf(fid,'<BR>\n');
         fprintf(fid,'<td>') ; fprintf(fid,'   AP:  '); fprintf(fid,'</td>') ; fprintf(fid,'\n');
         fprintf(fid,'<td>%4.2f</td>',aps); fprintf(fid, '\n <tr>\n') ;
%          fprintf(fid,'<BR>\n');
         fprintf(fid,'<td>') ;fprintf(fid,' Gain:  '); fprintf(fid,'</td>') ; fprintf(fid,'\n');
         fprintf(fid,'<td>%4.2f</td>',gain); fprintf(fid, '\n <tr> \n') ;
%          fprintf(fid,'<BR>\n');
%          fprintf(fid,'<BR>\n');
        fprintf(fid, '</table> ') ;
      end
   end
   
   fclose(fid);
   
end