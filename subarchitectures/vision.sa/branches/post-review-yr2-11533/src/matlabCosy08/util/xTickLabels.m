function xTickLabels(labels);

if ischar(labels(1))
   set(gca,'XtickLabel',labels);
   set(gca,'Xtick',1:size(labels,1));
else
   set(gca,'XtickLabel',num2str(labels'));
   set(gca,'Xtick',1:length(labels));
end;   
