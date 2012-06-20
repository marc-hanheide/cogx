function asvSave

global mAV mDA mFS NUs RSs IPs
global lbipH;
global ASVon ASVidx Dirs;


if ASVon
   
   ASVidx=ASVidx+1;

   IPs=get(lbipH,'String');

   fname=[Dirs.asv 'asv' num2str(ASVidx,'%04d') '.mat'];
   save(fname,'mAV','mFS','mDA','NUs','RSs','IPs');

end
