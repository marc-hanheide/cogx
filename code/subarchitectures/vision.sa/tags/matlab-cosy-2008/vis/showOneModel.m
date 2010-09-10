function showOneModel(hObject, eventdata, mC, Fnames, Cnames)

numd=length(mC.Fb);
numc=0;

figure;
set(gcf,'Name',(['mC: ' Cnames(mC.name,:) ' (' num2str(mC.conf) ')']));
set(gcf,'Position', [500, 500, 250, 400]);
set(gcf,'NumberTitle','off');

subplot(3,1,1:2);
ha=gca;

if ~isempty(mC.kde)
   executeOperatorIKDE( mC.kde, 'showKDE',  'selectSubDimensions', mC.Fb);
   numc=length(mC.kde.pdf.w);
end

fns=reshape(Fnames(mC.Fb,:)',1,numel(Fnames(mC.Fb,:)));
title(ha,['{\bf' Cnames(mC.name,:) '}\leftrightarrow' fns ' (' num2str(mC.conf) ')']);
% set(ha,'ytick',[]);
% alim=axis(ha);
% xticks=[alim(1),(alim(1)+alim(2))/2,alim(2)];
% set(ha,'XTick',xticks);
% set(ha,'XTickLabel',{num2str(xticks(1),'%3.3f'),num2str(xticks(2),'%3.3f'),num2str(xticks(3),'%3.3f')})
xlabel(Fnames(mC.Fb(1),:));
if numd>1
   ylabel(Fnames(mC.Fb(2),:));
end
if numd>2
   zlabel(Fnames(mC.Fb(3),:));
   rotate3d;
end


ha=subplot(3,1,3);
axis([0 100 0 100]);
axis off;
text(10,70,['Name: ' Cnames(mC.name,:)]);
fnst=[Fnames(mC.Fb,:)';repmat(' ',1,numd)];
fns=reshape(fnst,1,numel(fnst));
text(10,50,['Features: ' fns]);
text(10,30,['Dimensions: ' num2str(numd)]);
text(10,10,['Components: ' num2str(numc)]);
text(10,-10,['Confidence: ' num2str(mC.conf)]);
set(ha,'HitTest','off');

% 
% 
% global Fnames Cnames
% 
% figure;
% set(gcf,'name',([Cnames(mC.name,:) ' (' num2str(mC.conf) ')']));
% 
% ha=gca;
% 
% if ~isempty(mC.kde)
%    executeOperatorIKDE( mC.kde, 'showKDE',  'selectSubDimensions', mC.Fb);
% end
% 
% fns=reshape(Fnames(mC.Fb,:)',1,numel(Fnames(mC.Fb,:)));
% title(ha,['{\bf' Cnames(mC.name,:) '}\leftrightarrow' fns ' (' num2str(mC.conf) ')']);
% set(ha,'ytick',[]);
% alim=axis(ha);
% xticks=[alim(1),(alim(1)+alim(2))/2,alim(2)];
% set(ha,'XTick',xticks);
% set(ha,'XTickLabel',{num2str(xticks(1),'%3.3f'),num2str(xticks(2),'%3.3f'),num2str(xticks(3),'%3.3f')})
% xlabel(Fnames(mC.Fb(1),:));
% ylabel(Fnames(mC.Fb(2),:));
% zlabel(Fnames(mC.Fb(3),:));
% %drawnow;
% rotate3d;