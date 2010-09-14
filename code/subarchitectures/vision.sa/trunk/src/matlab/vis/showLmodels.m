function showLmodels(mC,Fnames,Cnames,sphw,hp)
%showLmodels(mC,Fnames,Cnames,sphw,hp)
%Show learned models

getc(mC,'info');

return; %Don't show anything for now...

%disp('Showing models...');

if nargin<4
   sphw=[2 ceil(length(mC)/2)];
end;
if nargin<5
   hp=gcf;
end;

if ~isempty(mC(1).name)
   
   %ha=subplot(1,1,1,'Parent',hp);
   %set(ha,'visible','Off');
   
   numC=length(mC);
   for i=1:numC
      numd=length(mC(i).Fb);
      
      ha=subplot(sphw(1),sphw(2),i,'Parent',hp);
      axes(ha);
      if ~isempty(mC(i).kde)
         %         executeOperatorIKDE( mC(i).kde, 'showKDE', 'selectSubDimensions', 1);
         executeOperatorIKDE( mC(i).kde, 'showKDE',  'selectSubDimensions', mC(i).Fb);
      end
      %      title(ha,['{\bf' Cnames(mC(i).name,:) '}\leftrightarrow' Fnames(mC(i).Fb,:) ' (' num2str(mC(i).conf) ')']);
      fns=reshape(Fnames(mC(i).Fb,:)',1,numel(Fnames(mC(i).Fb,:)));
      title(ha,['{\bf' Cnames(mC(i).name,:) '}\leftrightarrow' fns ' (' num2str(mC(i).conf) ')']);
      
      set(ha,'xtick',[]);
      set(ha,'ytick',[]);
      if numd>2
         set(ha,'ztick',[]);
      end
      %set(ha,'Box','Off');
      
      %       set(ha,'ytick',[]);
      %       set(ha,'xtick',[]);
      %       alim=axis(ha);
      %       xticks=[alim(1),(alim(1)+alim(2))/2,alim(2)];
      %       set(ha,'XTick',xticks);
      %       set(ha,'XTickLabel',{num2str(xticks(1),'%3.3f'),num2str(xticks(2),'%3.3f'),num2str(xticks(3),'%3.3f')})
      set(ha,'ButtonDownFcn',{'showOneModel',mC(i),Fnames,Cnames});
   end
end;
