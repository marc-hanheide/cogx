function [lrcH LRguiL LRguiR]=LRstart

global mC

mC=ODKDEinit;

lrcH=LRcontrol;
[LRguiL LRguiR]=LRvisStart;
%lrvH=LRvisStart;
%lreH=LRevalStart;

lrHs=[lrcH LRguiL LRguiR LRguiL LRguiR];