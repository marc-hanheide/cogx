function maxFigs(figs);

screen = get(0, 'ScreenSize');

nfigs=length(figs);
for i=1:nfigs
   set(figs(i),'Position',[1,1,screen(3),screen(4)-20]);
end;
