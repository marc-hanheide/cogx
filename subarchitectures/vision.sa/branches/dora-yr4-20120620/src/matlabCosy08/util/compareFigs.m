function compareFigs(figs,dmode);
%COMPAREFIGS  Display figures and enable visual comparison.
%   COMPAREFIGS(FIGS) compares figures with figure numbers listed in 
%   the row vector FIGS.
%   COMPAREFIGS compares all figures.
%   COMPAREFIGS(FIGS,DMODE) compares figures in different modes:
%      DMODE=0: Just displays figures as they are.
%      DMODE=1: Also aligns figures (default).
%      DMODE=2: Also aligns and maximizes figures.
%
%   During execution:
%     right: show next figure
%     left: show previous figure
%     up,key: assign the key to the current figures
%     key: show the figure previously assigned to the key
%     down: toggle lastly displayed figure
%     escape: exit
%     ctrl+w: close figures and exit
%
%   See also DINIT, DFIGURE, HIDEFIGS, SHOWFIGS, CLOSEFIGS, COMPOUNDFIGS.

if nargin==0 %compare all figures
   numfigs=figure;
   close(numfigs);
   numfigs=numfigs-1;
   figs=[1:numfigs];
end;   
if nargin<2 dmode=1; end;

nfigs=length(figs);

if dmode>0 
   %save positions
   pos0=zeros(nfigs,4);
   for i=1:nfigs
      pos0(i,:)=get(figs(i),'Position');
   end;      
   %align
   alignFigs(figs); 
end;   
if dmode==2 maxFIgs(figs); end; %maximize

NOTAG='¤';
for i=1:nfigs tags(i)=NOTAG;end 

loop=1;
setmode=0;
cfig=1;
lfig=1;
while loop
	showFigs(figs(cfig));
   waitforbuttonpress;
   ch=get(gcf,'CurrentCharacter');
	switch ch
   case '' %left
      lfig=cfig;
      cfig=cfig+1;
      if cfig>nfigs cfig=1; end;
   case '' %right
      lfig=cfig;
      cfig=cfig-1;
      if cfig<1 cfig=nfigs; end;
   case '' %up
      setmode=1;
   case '' %down
      lfig1=cfig;
      cfig=lfig;
      lfig=lfig1;
   case '' %esc
      loop=0;
   case '' %ctrl-w
      closefigs(figs);
      loop=0;
      dmode=0;
   otherwise
      if setmode  %set tag ch to cfig
			for i=1:nfigs
			   if tags(i)==ch tags(i)=NOTAG; end;
			end;
			tags(cfig)=ch;
         setmode=0;
      else  %go to figure with tag ch
         i=1;
         while i<=nfigs & ~isequal(tags(i),ch)
            i=i+1;
         end;
         if i<=nfigs
            lfig=cfig;
            cfig=i;
         end;   
      end;   
   end;       
end;  

if dmode>0
   for i=1:nfigs  %restore positions
      set(figs(i),'Position',pos0(i,:));
   end;
end;   
