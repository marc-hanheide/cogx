function addPaths(RDIR)

if nargin==0
%   RDIR='C:\danijels\Matlab\cogLearn';
   RDIR=pwd;
end;

addpath(genpath(RDIR));
addpath(RDIR);
