function asvLoad

global Settings

global IPs
global lbipH;
global Dirs

if Settings.ASVon

   %disp(Settings.Params.ASVidx);
   fname=[Dirs.asv 'asv' num2str(Settings.Params.ASVidx,'%04d') '.mat'];
   fid = fopen(fname);
   if fid==-1 %go to last
      l=dir([Dirs.asv 'asv*.mat']);
      if length(l)>0
         ln=l(end).name;
         Settings.Params.ASVidx=str2num(ln(4:7));
         fname=[Dirs.asv 'asv' num2str(Settings.Params.ASVidx,'%04d') '.mat'];
         fid = fopen(fname);
      end
   end;

   if fid~=-1
      fclose(fid);
      load(fname);

      LRvisUpdate;
      %LRevalUpdate;

      set(lbipH,'String',IPs);
      set(lbipH,'Value',size(IPs,1));
   end
end

