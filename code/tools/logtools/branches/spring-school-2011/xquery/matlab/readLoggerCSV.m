function data=readLoggerCSV(fname)
fid=fopen(fname);
raw=textscan(fid,'%n%s%s%s','Delimiter',',');
fclose(fid);

