function showLmodels(mC,Fnames,Cnames,sphw,fig)
%showLmodels(mC,Fnames,Cnames,sphw,hp)
%Show learned models

%getc(mC,'info');

hp=fig.pnModels;

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
   set(ha, 'box', 'on') ;
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

txt=strFromIdxs(mC);	
if length(txt)>1 && numCall>0%8
    a=axis(ha);
    w = a(2) - a(1) ;
    
    len_p = length([mC{1}.class_labels,mC{2}.class_labels]) ;
    sh = mod( len_p,5) ;
    dd = 0 ;
    if sh == 0
        sh = 4 ;
    end
    if sh > 2          
        dd = 4 + sh ;
    end
    if size(a) <= 4
        h = a(4) - a(3) ;
        text(a(1)- (dd*w/1),a(3)-0.2*h,txt,'FontSize',12 , 'Parent', ha);
    else
        h = a(4) - a(3) ;
        text(a(1)- (dd*w/1),0,a(5)-0.2*h,txt,'FontSize',12 , 'Parent', ha);
    end
    
    
end
if length(txt)>1%8
   set(fig.tx_Fb,'String',txt);
end
displayG(fig.main,'GL');
displayTL(mC);
%displayTD(mC);
displayGD(mC);




function txt=strFromIdxs(mC)

numSC=getc(mC,'numSC');

global Coma

txt=[];
for sc=1:numSC
txt=[txt '['];
   fb=getc(mC,sc,'Fb');
   %txt=[txt [Coma.Cnames(fb,:) ' ']];
   cs=getc(mC,sc,0,'name');
   sccs=cs';%Coma.SCC(Coma.SCC(cs,2)==sc,1);
   sccst=[Coma.Cnames(sccs,:) repmat(' ',size(sccs,1),1)];
   sccst1=reshape(sccst',1,numel(sccst));
   txt=[txt sccst1 ']: '];
   
   fbs=reshape(Coma.Fnames(fb,:)',1,numel(Coma.Fnames(fb,:)));
   txt=[txt fbs '     '];   
end
