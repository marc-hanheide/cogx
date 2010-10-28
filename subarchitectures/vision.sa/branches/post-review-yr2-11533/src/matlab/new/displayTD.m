function displayTD(mC)

global Coma Disp Settings

if Settings.Disp.TD
   fid=fopen(Disp.mTD,'w');
   
%    fprintf(fid,'Debug window!<BR>\n');
%    fprintf(fid,'<BR>\n');
%    
%    numSC=getc(mC,'numSC');
%    for sc=1:numSC
%       fprintf(fid,'<u>%s:</u> <BR>\n',Coma.SCnames(sc,:));
%       fprintf(fid,'cummulative_feat_costs: '); fprintf(fid,'%6.3f ',mC{sc}.cummulative_feat_costs); fprintf(fid,'<BR>\n');
%       fprintf(fid,'<BR>\n');
%    end
 
   fprintf(fid, '<font size="5" color="blue">%s</font>', mC);

   fclose(fid);
   
end