function asvSave

global Settings

global IPs
global lbipH;
global Dirs;


if Settings.ASVon
   
   Settings.Params.ASVidx=Settings.Params.ASVidx+1;

   IPs=get(lbipH,'String');

   fname=[Dirs.asv 'asv' num2str(Settings.Params.ASVidx,'%04d') '.mat'];
   save(fname,'mC','mFS','mDA','NUs','RSs','IPs');

end
