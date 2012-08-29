function asvSave

global Settings

global Figs
global Dirs
global mC


if Settings.ASVon
   
   Settings.Params.ASVidx=Settings.Params.ASVidx+1;

   %Settings.Params.IPs=get(Figs.vmipH.lbipH,'String');

   fname=[Dirs.asv 'asv' num2str(Settings.Params.ASVidx,'%04d') '.mat'];
   %save(fname,'mC','mFS','mDA','NUs','RSs','IPs');
   save(fname,'mC');

end
