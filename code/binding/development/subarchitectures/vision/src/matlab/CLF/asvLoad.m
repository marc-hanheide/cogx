function asvLoad

global mAV mDA mFS NUs RSs IPs
global lbipH;
global ASVidx;

if ASVidx<=0
   l=dir('asv/asv*.mat');
   ln=l(end).name;
   ASVidx=str2num(ln(4:7));
end;   
   
fname=['./asv/asv' num2str(ASVidx,'%04d') '.mat'];
load(fname);

LRvisUpdate;
LRevalUpdate;

set(lbipH,'String',IPs);
set(lbipH,'Value',size(IPs,1));