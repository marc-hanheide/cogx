function showRec(ansYes,ansPy,answ,f)
%function showRec(answ,f)

global Figs Coma Params
%global LRaxRoi LRtxFroi LRaxRec LRtxRec LRtxFrec LRtxFisRec
%global currMode

numC=size(Coma.Cnames,1);
THRs=Params.THRs;
MaxTyp=1;%min(1,THRs(1)*4);

if ~isempty(answ)

  recYes=idx2name(ansYes,Coma.Cnames);
  recPy=idx2name(ansPy,Coma.Cnames);

  str=recYes;
  if ~isempty(recPy)
     str=[str '    (' recPy ' )'];
  end
  set(Figs.LRguiR.LRtxRec,'String',str);

   bar(Figs.LRguiR.LRaxRec,answ(:,2),'b');
   set(Figs.LRguiR.LRaxRec,'XTickLabel',Coma.Cnames(answ(:,1),:));
   axis(Figs.LRguiR.LRaxRec,[0 numC+1 0 MaxTyp]);
   
line([0 numC+1],[THRs(1) THRs(1)],'Color','g','Parent',Figs.LRguiR.LRaxRec);
line([0 numC+1],[THRs(2) THRs(2)],'Color','m','Parent',Figs.LRguiR.LRaxRec);
%line([0 numC+1],[THRs(3) THRs(3)],'Color','r','Parent',LRaxRec);
   
   
end

set(Figs.LRguiR.LRtxFisRec,'Visible','on');
set(Figs.LRguiR.LRtxFrec,'String',[num2str(f','%.2g  ') ' ]']);


