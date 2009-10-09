function [LRguiL LRguiR]=LRstart

global mAV mDA mFS;

%[mAV,mDA,mFS]=MVBFinit;
[mAV,mDA,mFS]=KDBFinit;

lrcH=LRcontrol;
[LRguiL LRguiR]=LRvisStart;
%lrvH=LRvisStart;
%lreH=LRevalStart;

lrHs=[lrcH LRguiL LRguiR LRguiL LRguiR];