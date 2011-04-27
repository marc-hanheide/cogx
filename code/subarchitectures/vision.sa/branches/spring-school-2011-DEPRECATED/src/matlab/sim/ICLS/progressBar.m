function progressBar(ah,pctg)

barh(ah,pctg,'r','EdgeColor','r','barWidth',.2);
set(ah,'Xlim',[0 1]);
set(ah,'Ylim',[.9 1.1]);
set(ah,'XTick',[]);
set(ah,'YTick',[]);
drawnow;