function loadConfig(confFile)

fid = fopen(confFile);
if fid~=-1
   fclose(fid);
%   disp(['Loading cogLearn config file ' confFile ' .']);
   disp(['Loading config file ' confFile ' .']);
   lines = textread(confFile, '%s', 'delimiter', '\n', 'whitespace', '');
   for i=1:length(lines)
      eval(lines{i})
   end
end

if 0
   showOneModel();
end;
