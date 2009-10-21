function showLogRes(Ns,Rs)

%for 81 NNs (1:400)
lns=[1:3 unique(round(exp(5:.1:8)/40)) 81];
tns=1:5:length(lns);

dfigure(2,2);

setFonts(14);
plot(Rs(lns,:),'LineWidth',2)
%set(gca,'LineWidth',2);
set(gca,'XTick',tns);
set(gca,'XTickLabel',Ns(lns(tns)));
setAxis(1,length(lns));
legend('TD','TS','TA','TU',4);
xlabel('Number of images');
resizeFigs(gcf,2,2);


return;

Ns=squeeze(MTTRES(1,:,:));
RSs=squeeze(MTTRES(2,:,:));
TCs=ttc2tc(squeeze(MTTRES(1,:,:)),squeeze(MTTRES(8,:,:)));TCs(1:3,:)=Inf;
showLogRes(Ns,RSs);
ylabel('Recognition score');
showLogRes(Ns,TCs);
ylabel('Tutoring cost');

print -dpdf expEpirob1RS
print -dpdf expEpirob1TC
