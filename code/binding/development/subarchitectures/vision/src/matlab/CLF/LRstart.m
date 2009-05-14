function lrHs=LRstart

global mAV mDA mFS;

%[mAV,mDA,mFS]=MVBFinit;
[mAV,mDA,mFS]=KDBFinit;

lrcH=LRcontrol;
lrvH=LRvisStart;
%lreH=LRevalStart;

lrHs=[lrcH lrvH];