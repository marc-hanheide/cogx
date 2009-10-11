function printRes(res,msg,disps)
%printRes(res,msg,disps);
%Print results.
%res: results
%msg: message that appears in front of results.
%disps: a string or a cell array of strings with names of the evaluation data to
%       display (for every row of the results individualy)

if nargin>1
   fprintf(msg);
end;

fprintf('rs=%5.0f (max.%5.0f)  rr1=%5.2f rr2=%5.2f  tpf1=%4.2f tnf1=%4.2f  tpf2=%4.2f tnf2=%4.2f\n' ,...
   res.rs,res.rsmax,res.rr1,res.rr2,res.tpf1,res.tnf1,res.tpf2,res.tnf2);

if nargin>2

   if ~iscell(disps)
      tdisps=disps;
      disps=cell(1,1);
      disps{1}=tdisps;
   end;   
   nc=length(res.rr1s);
   
   for j=1:length(disps)
      cres=eval(['res.' disps{j}]);
      fprintf('%s %s=[',repmat(' ',1,length(msg)-1),disps{j});
      for i=1:nc-1
         fprintf('%5.2f ',cres(i));
      end;
      fprintf('%5.2f]\n',cres(nc));
   end
end
