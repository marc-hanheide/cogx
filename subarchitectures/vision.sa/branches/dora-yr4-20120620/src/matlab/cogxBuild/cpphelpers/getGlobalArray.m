function res=getGlobalArray(glob, var)
   eval(['global ' glob ';']);
   res = eval([var ';']);
