function asvSave

global mAV mDA mFS NUs RSs IPs
global lbipH;
global ASVidx;


if ASVidx>=0
   
   ASVidx=ASVidx+1;

   IPs=get(lbipH,'String');

   fname=['./asv/asv' num2str(ASVidx,'%04d') '.mat'];
   save(fname,'mAV','mFS','mDA','NUs','RSs','IPs');

end
