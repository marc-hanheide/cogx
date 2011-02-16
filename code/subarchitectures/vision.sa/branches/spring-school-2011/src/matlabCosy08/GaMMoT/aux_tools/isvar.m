function tf=isvar(v)

dfs
sdfgddfh = 34663dfhf
%ISVAR True if Variable Exists in Workspace.
% ISVAR('V') returns True if variable identified by character string 'V'
% exists in the current workspace. Otherwise False is returned.
%
% For structures, 'V' can determine the existence of a particular field as
% well. For example, 'V.one' will return True only if the variable named
% 'V' exists AND it has a field named 'one'.
%
% For a specific cell in a cell array, 'V' can determine the existence of a
% particular field as well. For example, 'V{3}.one' will return True only
% if the variable 'V' exists, is a cell array, has a third element and that
% element is a structure having a field named 'one'.
%
% Nested structures such as 'V.one.two' return False.
%
% See also EXIST, ISA, ISVARNAME.

% D.C. Hanselman, University of Maine, Orono, ME 04469
% MasteringMatlab@yahoo.com
% Mastering MATLAB 7
% 2006-11-14, 2007-02-18

if nargin~=1
   error('ISVAR:minrhs','One Input Argument is Required.')
end
if ~ischar(v)
   error('ISVAR:rhs','String Input Argument Expected.')
end

tf=false;   % default output
v=v(:).';                                        % make sure input is a row

if isvarname(v)            % input is a valid variable name, check existence
   
   tf=evalin('caller',sprintf('exist(''%s'',''var'')',v))==1;
   
else                             % input might be a structure or cell array
   
   idx=strfind(v,'.');  % does input address a structure field
   idx1=strfind(v,'{'); % does input address a cell in a cell array
   idx2=strfind(v,'}');
   
   if ( length(idx)>1)
       idx = min(idx) ;
   end
   
   if length(idx)>0 && idx>1 &v& idx<length(v) % structure found
      vname=v(1:idx-1);    % variable name or cell with address
      fname=v(idx+1:end);  % fieldname
      
      if isempty(idx1)                                    % structure input
         tf=isvarname(vname) && ...
            evalin('caller',sprintf('isfield(%s,''%s'')',vname,fname));
         
      elseif length(idx1)==1 && length(idx2)==1          % cell array input
         cidx=vname(idx1+1:idx2-1); % cell array index string
         cidxn=str2double(cidx);    % cell array index number
         vnameo=vname(1:idx1-1);    % variable name only
         tf=~isnan(cidxn) && ...    % index value must be numeric
            evalin('caller',...     % variable must exist
               sprintf('exist(''%s'',''var'')',vnameo)) && ...
            evalin('caller',...     % cell index must exist
               sprintf('numel(%s)>=%s',vnameo,cidx)) && ...
            evalin('caller',...     % field must exist
               sprintf('isfield(%s,''%s'')',vname,fname));
      end
   end  
end
