%function [lrcH LRguiL LRguiR]=LRstart
function [lrcH]=LRstart

global mC

mC=ODKDEinit;

lrcH=LRcontrol;
LRvisStart;
%lrvH=LRvisStart;
%lreH=LRevalStart;

lrHs=[lrcH];