function showLmodels(mC,Fnames,Cnames,sphw,hp)
%showLmodels(mC,Fnames,Cnames,sphw,hp)
%Show learned models

%getc(mC,'info');


if nargin<4
   sphw=[2 ceil(length(mC)/2)];
end;
if nargin<5
   hp=gcf;
end;

numCall=getc(mC,'numC');
cs=zeros(numCall,3);
j=0;
for sc=1:getc(mC,'numSC')
   for i=1:getc(mC,sc,'numC')
      j=j+1;
      cs(j,:)=[sc i getc(mC,sc,i,'name')];
   end
end

%ha=subplot(1,1,1,'Parent',hp);
%set(ha,'visible','Off');
for i=1:numCall
   sc=cs(i,1);
   ci=cs(i,2);
   c=cs(i,3);
   
   numd=1;%length(mC(i).Fb);
   
   ha=subplot(sphw(1),sphw(2),i,'Parent',hp);
%  plot(ha,1,1,'*b')
   
%    axes(ha);
%    plot(ha,1,1,'*')
   
%    if getc(mC,sc,ci,'conf')>6%~isempty(mC(i).kde)
%       executeOperatorIKDEClsfr( mC{sc}, 'showKDE_of_class_index', ci, 'sub_selected_features', 1, 'showkdecolor', 'r') ;
executeOperatorIKDEClsfr( mC{sc}, 'showKDE_of_class_index', ci, 'showkdecolor', 'r', 'draw_to_these_axes', ha) ;
%    end
   title(ha,['{\bf' Cnames(getc(mC,sc,ci,'name'),:) '} (' num2str(getc(mC,sc,ci,'conf')) ')']);
   fns=[];%reshape(Fnames(mC(i).Fb,:)',1,numel(Fnames(mC(i).Fb,:)));
   %      title(ha,['{\bf' Cnames(mC(i).name,:) '}\leftrightarrow' fns ' (' num2str(mC(i).conf) ')']);
   
   set(ha,'xtick',[]);
   set(ha,'ytick',[]);
   if numd>2
      set(ha,'ztick',[]);
   end
   %   set(ha,'ButtonDownFcn',{'showOneModel',mC(i),Fnames,Cnames});
end


dispLearn(get(hp,'Parent'));
